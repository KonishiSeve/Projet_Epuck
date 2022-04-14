#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>

#include <process_image.h>


static float distance_cm = 0;
static uint16_t centre = 0;

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
	uint32_t start = 0;

    while(1){
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
		//chprintf((BaseSequentialStream *)&SDU1, "time = %d \n", chVTGetSystemTime()-start);
		start = chVTGetSystemTime();
    }
}


static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		uint8_t img_buff[640] = {0};
		uint8_t* pointeur = &img_buff;
		uint64_t moyenne = 0;
		for(int i = 0; i<640;i++){
			uint8_t temp1 = img_buff_ptr[2*i+1];
			uint8_t temp2 = img_buff_ptr[2*i];
			temp1 &= 0b11100000;
			temp1 = temp1 >> 5;
			temp2 &= 0b00000111;
			temp2 = temp2 << 3;
			pointeur[i] = temp1 + temp2;
			moyenne += temp1 + temp2;
		}
		moyenne /= 640;
		uint16_t limGauche = 0;
		uint32_t largeur = 0;
		uint8_t threshold = moyenne / 1.3;
		uint32_t largeurMax = 0;
		for(int i = 0; i<640;i++){
			if(pointeur[i]<threshold && largeur == 0){
				largeur = i;
				limGauche = i;
				//chprintf((BaseSequentialStream *)&SDU1, "limGauche = %d \n", limGauche);
			}
			if(pointeur[i]>threshold && largeur != 0){
				largeur = i - largeur;
				if(largeur > largeurMax){
					largeurMax = largeur;
					centre = (limGauche + i )/2;
					//chprintf((BaseSequentialStream *)&SDU1, "centre = %d \n", get_centre());
				}
				largeur = 0;
			}
		}
		//chprintf((BaseSequentialStream *)&SDU1, "largeurMax = %d \n", largeurMax);
		distance_cm = largeurMax / 11.7;
		//chprintf((BaseSequentialStream *)&SDU1, "distance = %f \n", distance_cm);
		SendUint8ToComputer(pointeur, 640);
		chThdSleepMilliseconds(100);
		}
}

uint16_t get_centre(void){
	return centre;
}

float get_distance_cm(void){
	return distance_cm;
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
