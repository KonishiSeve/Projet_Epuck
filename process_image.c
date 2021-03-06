#include "ch.h"
#include "hal.h"

#include <camera/po8030.h>
#include <leds.h>

#include <calibration.h>
#include <main.h>
#include <process_image.h>

#define IMAGE_WIDTH 640
#define STATE_DAY 0
#define STATE_NIGHT 1

//Semaphore pour signaler la capture d'une image
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

//Thread pour capturer une image
static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//On capture les deux premieres lignes en partant du haut
    po8030_advanced_config(FORMAT_RGB565, 0, 0, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}

//variables pour communiquer avec le thread "navigation"
static uint16_t traffic_light_center = 0;
uint16_t get_traffic_light_center(void) {
	return traffic_light_center;
}
static uint16_t traffic_light_size = 0;
uint16_t get_traffic_light_size(void) {
	return traffic_light_size;
}
static uint8_t general_state = STATE_ROAD;
uint8_t get_general_state(void) {
	return general_state;
}

//Thread pour l'analyse de l'image capturee et machine d'etat principale
static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//buffer utilise pour stocker une ligne de pixels rouges
	uint8_t img_red[IMAGE_BUFFER_SIZE] = {0};

	//pour lire l'image de la camera
	uint8_t *img_buff_ptr;

	//Compteurs utilises pour declencher le suivit de feu rouge ou la conduite de nuit
	uint8_t trigger_red = 0;
	uint8_t trigger_night = 0;
	uint8_t day_night_state = STATE_DAY;

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		// ===== Separation des 3 cannaux de couleur et calcul des moyennes =====
		uint16_t mean_red = 0;
		uint16_t mean_green = 0;
		uint16_t mean_blue = 0;
		uint16_t green_mean_peak = 0;
		for(uint16_t i = 0; i<IMAGE_WIDTH;i++){
			uint8_t pixel_low = img_buff_ptr[2*i+1];
			uint8_t pixel_high = img_buff_ptr[2*i];
			uint8_t pixel_blue = pixel_low & 0b00011111;
			uint8_t pixel_green = (pixel_low >> 5) + ((pixel_high & 0b00000111) << 3);
			uint8_t pixel_red = pixel_high >> 3;

			img_red[i] = pixel_red;

			//Calcul de la moyenne verte dans le pic rouge du cycle precedent
			if(i >= (traffic_light_center-traffic_light_size/2) && i <= (traffic_light_center+traffic_light_size/2)) {
				green_mean_peak += pixel_green;
			}
			mean_red += pixel_red;
			mean_blue += pixel_blue;
			mean_green += pixel_green;
		}
		green_mean_peak /= traffic_light_size;
		mean_red = (mean_red/IMAGE_WIDTH) << 1; //conversion en format 6 bits
		mean_green = (mean_green/IMAGE_WIDTH);
		mean_blue = (mean_blue/IMAGE_WIDTH) << 1; //conversion en format 6 bits

		// ===== Detection de jour/nuit =====
		if(mean_blue < NIGHT_THRESHOLD){
			trigger_night += STEP_NIGHT;
		}
		else if(trigger_night >= STEP_DAY){
			trigger_night -= STEP_DAY;
		}
		if(trigger_night >= NIGHT_TRIGGER_THRESHOLD && general_state == STATE_ROAD) {
			set_front_led(1);
			trigger_night = NIGHT_TRIGGER_THRESHOLD;
			day_night_state = STATE_NIGHT;
			general_state = STATE_ROAD;
		}
		else if(trigger_night == 0){
			set_front_led(0);
			day_night_state = STATE_DAY;
		}

		//On ne fait pas les calculs de detection de pic en conduite de nuit
		if(day_night_state == STATE_DAY) {

			// ========== Detection de pic pour le feux rouge ==========
			uint16_t red_peak_left_limit = 0;
			uint16_t red_peak_width_max = 0;
			uint16_t red_peak_center = 0;
			for(uint16_t i = RED_SLOPE_SHARPNESS; i<IMAGE_WIDTH;i++){
				//Recherche du debut d'un pic
				if(img_red[i] > mean_red && img_red[i-RED_SLOPE_SHARPNESS] < mean_red) {
					if(red_peak_left_limit == 0) {
						red_peak_left_limit = i;
					}
				}
				//Recherche de la fin d'un pic
				else if(img_red[i] < mean_red && img_red[i-RED_SLOPE_SHARPNESS] > mean_red && red_peak_left_limit !=0) {
					//On prend le pic le plus large
					if((i - red_peak_left_limit - RED_SLOPE_SHARPNESS) > red_peak_width_max) {
						red_peak_width_max = i - red_peak_left_limit - RED_SLOPE_SHARPNESS;
						red_peak_center = (red_peak_left_limit+i-RED_SLOPE_SHARPNESS)/2;
					}
					red_peak_left_limit = 0;
				}
			}
			traffic_light_size = red_peak_width_max;
			traffic_light_center = red_peak_center;

			// ===== Calcul de la moyenne rouge dans le pic rouge =====
			uint16_t red_peak_mean = 0;
			for(uint16_t i=red_peak_left_limit;i<red_peak_left_limit+red_peak_width_max;i++) {
				red_peak_mean += img_red[i];
			}
			red_peak_mean /= red_peak_width_max;

			// ===== Calcul de l'ecart type rouge dans le pic rouge =====
			uint16_t red_peak_std = 0;
			for(uint16_t i=red_peak_left_limit;i<red_peak_left_limit+red_peak_width_max;i++) {
				red_peak_std += abs(img_red[i] - red_peak_mean);
			}
			red_peak_std = 10*red_peak_std/red_peak_width_max; //multiplication par 10 pour garder une precision sans utiliser de float

			// ========== Detection de feu rouge ==========
			if(general_state == STATE_ROAD && trigger_red < RED_PEAK_TRIGGER && mean_red >= RED_MEAN_THRESHOLD && red_peak_std >= RED_STD_THRESHOLD_LOW && red_peak_std <= RED_STD_THRESHOLD_HIGH && traffic_light_size >= RED_PEAK_WIDTH_THRESHOLD) {
				trigger_red++;
			}
			else {
				trigger_red = 0;
			}
			if(general_state == STATE_ROAD && trigger_red >= RED_PEAK_TRIGGER) {
				general_state = STATE_TRAFFIC_LIGHT;
				trigger_red = 0;
			}

			// ========== Detection de feu vert ===========
			if(general_state == STATE_TRAFFIC_LIGHT && green_mean_peak >= GREEN_MEAN_THRESHOLD) {
				general_state = STATE_ROAD;
			}
		}

		chThdSleepMilliseconds(100);
    }
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
