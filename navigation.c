#include "ch.h"
#include "hal.h"
#include <motors.h>
#include <main.h>

//Thread pour éviter les obstacles <-> s'aligner avec le feu
// sleep 200ms
//envoie à clignotant
//reçoit de process_image si le feu est vert ou rouge
//reçoit les données des capteurs infrarouge

static uint8_t clignoter;

uint8_t get_cligno(){
	return clignoter;
	}

THD_WORKING_AREA(navigation_thd_wa, 256);
THD_FUNCTION(navigation_thd,arg) {

	chRegSetThreadName(__FUNCTION__);
	(void) arg;
	//int16_t speedk = 200;
	//int16_t speedk2 = 200;
	int16_t diffspeed = 0;

	while(1) {
		while(1) {
			clignoter = 0;
			// éviter obstacle()
			const int8_t Kp = 1;
			int16_t proxLeft = get_prox(7) + get_prox(6);
			int16_t proxRight = get_prox(0) + get_prox(1);

			int16_t diff = proxLeft - proxRight;
/*
			speedk = diff * Kp;
			speedk2 =  - diff * Kp;*/
			diffspeed = diff*Kp;

			left_motor_set_speed(200 + diffspeed);
			right_motor_set_speed(200 - diffspeed);

			if(diffspeed > 50){
				clignoter = 1;
			}else if(diffspeed < -50){
				clignoter = 2;
			}
			chThdSleepMilliseconds(100);
		}
		//attend_que_le_feu_soit_vert();
	}
}

void navigation_start(void) {
	chThdCreateStatic(navigation_thd_wa, sizeof(navigation_thd_wa), NORMALPRIO, navigation_thd, NULL);
}

