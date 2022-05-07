#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>

#include <chprintf.h>
#include <process_image.h>

#include <leds.h>

#define NIGHT_THRESHOLD 20
#define RED_THRESHOLD 20
#define TAILLE_FEU_THRESHOLD 150
#define THRESHOLD_GREEN 20

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
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

static uint16_t centre_feu = 0;
uint16_t get_centre_feu(void) {
	return centre_feu;
}
static uint16_t taille_feu = 0;
uint16_t get_taille_feu(void) {
	return taille_feu;
}

//0: route, 1:suivit feu, 2:attente feu
static uint8_t general_state = 0;
uint8_t get_general_state(void) {
	return general_state;
}


static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};

	uint8_t trigger_red = 0;
	uint16_t mean_green_old = 0;

	uint16_t debug_counter = 0;

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		uint8_t img_buff[640] = {0};
		uint8_t* pointeur_image = &image;

		//-------------------------CALCULS----------------------------
		uint64_t moyenne_red = 0;
		uint64_t moyenne_green = 0;
		uint64_t moyenne_blue = 0;
		for(int i = 0; i<640;i++){
			uint8_t pixel_low = img_buff_ptr[2*i+1];
			uint8_t pixel_high = img_buff_ptr[2*i];
			uint8_t pixel_blue = pixel_low & 0b00011111;
			uint8_t pixel_green = (pixel_low >> 5) + ((pixel_high & 0b00000111) << 3);
			uint8_t pixel_red = pixel_high >> 3;
			pointeur_image[i] = pixel_red;
			moyenne_red += pixel_red;
			moyenne_blue += pixel_blue;
			moyenne_green += pixel_green;

		}
		moyenne_red = (moyenne_red/640) << 1; //conversion en 6 bits
		moyenne_green = (moyenne_green/640);
		moyenne_blue = (moyenne_blue/640) << 1; //conversion en 6 bits


		//------------------Détection de pic pour le feux rouge--------------------------------
		uint16_t threshold_red = moyenne_red/1.3;
		uint16_t largeur_pic = 0;
		uint16_t limite_gauche_pic = 0;
		uint16_t largeur_max = 0;
		uint16_t centre_pic = 0;

		for(int i = 0; i<640;i++){
			if(pointeur_image[i] > threshold_red && largeur_pic==0) {
				largeur_pic = i;
				limite_gauche_pic = i;
			}
			if(pointeur_image[i] < threshold_red && largeur_pic !=0 ) {
				largeur_pic = i-largeur_pic;
				if(largeur_pic > largeur_max) {
					largeur_max = largeur_pic;
					centre_pic = (limite_gauche_pic+i)/2;
				}
				largeur_pic = 0;
			}
		}

		//calcul de l'écart type seulement dans la région du pic rouge
		uint64_t standev = 0;
		for(int i=limite_gauche_pic; i<(limite_gauche_pic+largeur_max); i++) {
			standev += abs(pointeur_image[i]-moyenne_red);
		}
		standev /= 100;
		taille_feu = largeur_max;
		centre_feu = centre_pic;

		chprintf((BaseSequentialStream *)&SD3, "taille: %d", taille_feu);
		chprintf((BaseSequentialStream *)&SD3, " , centre: %d", centre_feu);
		chprintf((BaseSequentialStream *)&SD3, " , moyenne: %d", moyenne_red);
		chprintf((BaseSequentialStream *)&SD3, " , moyenne vert: %d", moyenne_green);
		chprintf((BaseSequentialStream *)&SD3, " , std: %d \r \n", standev);


		//------------------------- GESTION DU STATE ET ENVOI DES VARIABLES --------------------------------
		if(standev < 10 && standev > 0 && moyenne_red >= 20 && trigger_red < 3 && general_state == 0) {
			trigger_red++;
		} else if(trigger_red>=3 && general_state==0) {
			general_state = 1;
			trigger_red = 0;
			chprintf((BaseSequentialStream *)&SD3, "----- RED TRIGGER-----");
		} else {
			trigger_red = 0;
		}

		if(general_state == 1) {
			debug_counter++;
		}

		if(debug_counter >= 40) {
			general_state = 0;
			debug_counter = 0;
		}

		/*
		if(standev < 10 && standev >= 0 && moyenne_red >= 20 && trigger_red < 3) {
			trigger_red++;
		}

		if(moyenne_green > mean_green_old+5 && trigger_red >=4) {
			taille_feu = 0;
			centre_feu = 0;
			trigger_red = 0;
		} else if(trigger_red >=3) {
			taille_feu = largeur_max;
			centre_feu = centre_pic;
			mean_green_old = moyenne_green;
			chprintf((BaseSequentialStream *)&SD3, "RED TRIGGER");
			trigger_red++;

		} else {
			taille_feu = 0;
			centre_feu = 0;
		}*/


		if((moyenne_green + moyenne_blue < NIGHT_THRESHOLD - 5)){
			//set_front_led(1);
		}else if(moyenne_green + moyenne_blue > NIGHT_THRESHOLD + 60){
			set_front_led(0);
		}
		chThdSleepMilliseconds(100);
    }
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
