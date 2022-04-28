#include "ch.h"
#include "hal.h"
#include <motors.h>
#include <main.h>

//Thread pour éviter les obstacles <-> s'aligner avec le feu
// sleep 200ms
//envoie à clignotant
//reçoit de process_image si le feu est vert ou rouge
//reçoit les données des capteurs infrarouge

THD_WORKING_AREA(navigation_thd_wa, 256);
THD_FUNCTION(navigation_thd,arg) {

	chRegSetThreadName(__FUNCTION__);
	(void) arg;
	int16_t speedk = 200;
	int16_t speedk2 = 200;

	while(1) {
		while(1) {
			// éviter obstacle()

			const int8_t Kp = 1;
			int16_t proxLeft = get_prox(7) + get_prox(6);
			int16_t proxRight = get_prox(0) + get_prox(1);

			int16_t diff = proxLeft - proxRight;

			speedk = diff * Kp;
			speedk2 =  - diff * Kp;

			left_motor_set_speed(200 + speedk);
			right_motor_set_speed(200 + speedk2);

			chThdSleepMilliseconds(200);
		}
		//attend_que_le_feu_soit_vert();
	}
}

void navigation_start(void) {
	chThdCreateStatic(navigation_thd_wa, sizeof(navigation_thd_wa), NORMALPRIO, navigation_thd, NULL);
}

