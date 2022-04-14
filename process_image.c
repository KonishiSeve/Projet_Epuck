#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>

#include <process_image.h>
#include <leds.h>


static float distance_cm = 0;
static uint16_t centre = 0;

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 0, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
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
		uint64_t moyenne_red = 0;
		uint64_t moyenne_green = 0;
		uint64_t moyenne_blue = 0;

		for(int i = 0; i<640;i++){
			uint8_t pixel_low = img_buff_ptr[2*i+1];
			uint8_t pixel_high = img_buff_ptr[2*i];

			uint8_t pixel_blue = pixel_low & 0b00011111;
			uint8_t pixel_green = (pixel_low >> 5) + ((pixel_high & 0b00000111) << 3);
			uint8_t pixel_red = pixel_high >> 3;

			moyenne_red += pixel_red;
			moyenne_green += pixel_green;
			moyenne_blue += pixel_blue;

			/*
			temp1 &= 0b11100000;
			temp1 = temp1 >> 5;
			temp2 &= 0b00000111;
			temp2 = temp2 << 3;
			pointeur[i] = temp1 + temp2;
			moyenne += temp1 + temp2;
			*/
		}
		moyenne_red = (moyenne_red/640) << 1; //conversion en 6 bits
		moyenne_green = (moyenne_green/640);
		moyenne_blue = (moyenne_blue/640) << 1; //conversion en 6 bits

		//chprintf((BaseSequentialStream *)&SD3, "pixel_mean = %d \n", (moyenne_red + moyenne_blue)/2);
		if((moyenne_red + moyenne_blue)/2 < NIGHT_THRESHOLD) {
			set_front_led(1);
		} else {
			set_front_led(0);
		}
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
