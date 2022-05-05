#include "ch.h"
#include "hal.h"
#include <motors.h>
#include <main.h>
#include <process_image.h>
#include <chprintf.h>
#include <leds.h>

#define CENTRE_IMAGE 320
#define KP_FEU 2
//Thread pour éviter les obstacles <-> s'aligner avec le feu
// sleep 200ms
//envoie à clignotant
//reçoit de process_image la position du centre/la taille du feu (les deux == 0 si pas de feu détecté)
//reçoit les données des capteurs infrarouge


//utilisé pour communiquer avec le thread clignotant
static uint8_t clignoter;
uint8_t get_cligno(){
	return clignoter;
}


//thread qui s'occupe d'éviter les obstacles et suivre le feu
THD_WORKING_AREA(navigation_thd_wa, 256);
THD_FUNCTION(navigation_thd,arg) {

	chRegSetThreadName(__FUNCTION__);
	(void) arg;
	int16_t diffspeed = 0;

	while(1) {
		//Mode suivit de route
		while(0/*get_taille_feu()==0*/) {
			//chprintf((BaseSequentialStream *)&SD3, "taille: %d \r \n", get_taille_feu());
			clignoter = 0;
			// éviter obstacle()
			const int8_t Kp = 1;
			int16_t proxLeft = get_prox(7) + get_prox(6);
			int16_t proxRight = get_prox(0) + get_prox(1);

			int16_t diff = proxLeft - proxRight;
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
		set_rgb_led(LED4, 99,0,0);
		set_rgb_led(LED6, 99,0,0);
		set_led(LED5,2);
		//Mode alignement avec le feu et attente du feu vert
		while(1/*get_taille_feu() != 0*/) {
			chprintf((BaseSequentialStream *)&SD3, "centre: %d \r \n", get_centre_feu());
				int16_t erreur = get_centre_feu() - CENTRE_IMAGE;
				left_motor_set_speed(150 + KP_FEU*erreur);
				right_motor_set_speed(150 - KP_FEU*erreur);
		}
		set_rgb_led(LED4, 0,0,0);
		set_rgb_led(LED6, 0,0,0);
		set_led(LED5,0);
	}
}
void navigation_start(void) {
	chThdCreateStatic(navigation_thd_wa, sizeof(navigation_thd_wa), NORMALPRIO, navigation_thd, NULL);
}

