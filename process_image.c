#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>

#include <chprintf.h>
#include <process_image.h>

#include <leds.h>

#include <calibration.h>

#define IMAGE_WIDTH 640

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 0 + 1 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
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


static THD_WORKING_AREA(waProcessImage, 2048);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t img_red[IMAGE_BUFFER_SIZE] = {0};
	uint8_t img_green[IMAGE_BUFFER_SIZE] = {0};
	uint8_t img_blue[IMAGE_BUFFER_SIZE] = {0};
	uint8_t* img_red_ptr = &img_red;
	uint8_t* img_green_ptr = &img_green;
	uint8_t* img_blue_ptr = &img_blue;

	uint8_t trigger_red = 0;
	uint8_t trigger_green = 0;


    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		uint8_t img_buff[IMAGE_WIDTH] = {0};

		uint64_t mean_red = 0;
		uint64_t mean_green = 0;
		uint64_t mean_blue = 0;
		//Separation et Stockage des 3 cannaux de couleur, calcul des moyennes
		for(int i = 0; i<IMAGE_WIDTH;i++){
			uint8_t pixel_low = img_buff_ptr[2*i+1];
			uint8_t pixel_high = img_buff_ptr[2*i];
			uint8_t pixel_blue = pixel_low & 0b00011111;
			uint8_t pixel_green = (pixel_low >> 5) + ((pixel_high & 0b00000111) << 3);
			uint8_t pixel_red = pixel_high >> 3;

			img_red_ptr[i] = pixel_red << 1; //conversion en 6 bits
			img_green_ptr[i] = pixel_green;
			img_blue_ptr[i] = pixel_blue; //conversion en 6 bits

			mean_red += pixel_red;
			mean_blue += pixel_blue;
			mean_green += pixel_green;
		}
		mean_red = (mean_red/IMAGE_WIDTH) << 1; //conversion en 6 bits
		mean_green = (mean_green/IMAGE_WIDTH);
		mean_blue = (mean_blue/IMAGE_WIDTH) << 1; //conversion en 6 bits


		//Detection de pic pour le feux rouge
		uint16_t threshold_red = mean_red*RED_PEAK_THRESHOLD_COEFF;
		uint16_t threshold_green = mean_green*GREEN_PEAK_THRESHOLD_COEFF;
		uint16_t threshold_blue = mean_blue*BLUE_PEAK_THRESHOLD_COEFF;

		uint16_t peak_left_limit = 0;
		uint16_t peak_width_max = 0;
		uint16_t peak_center = 0;

		for(int i = 0; i<IMAGE_WIDTH;i++){
			//Recherche d'un pic rouge
			if(general_state == STATE_ROAD && img_red_ptr[i] > threshold_red && img_green_ptr[i] < threshold_green && img_blue_ptr[i] < threshold_blue) {
				if(peak_left_limit==0) {
					peak_left_limit = i;
				}
			}
			//Recherche d'un pic vert
			else if(general_state == STATE_TRAFFIC_LIGHT && img_red_ptr[i] < threshold_red && img_green_ptr[i] > threshold_green && img_blue_ptr[i] < threshold_blue) {
				if(peak_left_limit==0) {
					peak_left_limit = i;
				}
			}
			else if(peak_left_limit !=0) {
				if((i - peak_left_limit) > peak_width_max) {
					peak_width_max = i - peak_left_limit;
					peak_center = (peak_left_limit+i)/2;
				}
				peak_left_limit = 0;
			}
		}
		traffic_light_size = peak_width_max;
		traffic_light_center = peak_center;

		//Detection de feu rouge
		if(general_state == STATE_ROAD && peak_width_max > PEAK_WIDTH_THRESHOLD && trigger_red < RED_PEAK_TRIGGER) {
			trigger_red++;
		}
		if(general_state == STATE_ROAD && trigger_red >= PEAK_TRIGGER) {
			general_state == STATE_TRAFFIC_LIGHT;
			trigger_red = 0;
		}

		//Detection de feu vert
		if(general_state == STATE_TRAFFIC_LIGHT && peak_width_max > PEAK_WIDTH_THRESHOLD && trigger_green < GREEN_PEAK_TRIGGER) {
			trigger_green++;
		}
		if(general_state == STATE_ROAD && trigger_red >= PEAK_TRIGGER) {
			general_state == STATE_ROAD;
			trigger_green = 0;
		}

		//Detection de jour/nuit
		if(moyenne_blue < NIGHT_THRESHOLD){
			//set_front_led(1);
		}else if(moyenne_blue > DAY_THRESHOLD){
			set_front_led(0);
		}

		//Pour la calibration
		chprintf((BaseSequentialStream *)&SD3, "taille: %d", traffic_light_size);
		chprintf((BaseSequentialStream *)&SD3, " , centre: %d", traffic_light_center);
		chprintf((BaseSequentialStream *)&SD3, " , mean red: %d", mean_red);
		chprintf((BaseSequentialStream *)&SD3, " , mean vert: %d", mean_green);
		chprintf((BaseSequentialStream *)&SD3, " , mean blue: %d", mean_blue);

		chThdSleepMilliseconds(100);
    }
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
