#include "ch.h"
#include "hal.h"
#include <motors.h>
#include <main.h>
#include <process_image.h>
#include <chprintf.h>
#include <leds.h>
#include <calibration.h>
#include <navigation.h>

#define CENTRE_IMAGE 320
#define CIBLE_TAILLE 240
#define KP_FEU 1

//Thread pour �viter les obstacles <-> s'aligner avec le feu
// sleep 200ms
//envoie � clignotant
//re�oit de process_image la position du centre/la taille du feu (les deux == 0 si pas de feu d�tect�)
//re�oit les donn�es des capteurs infrarouge


//utilis� pour communiquer avec le thread clignotant
static uint8_t clignoter = BLINK_OFF;
uint8_t get_cligno(){
	return clignoter;
}


//thread qui s'occupe d'�viter les obstacles et suivre le feu
THD_WORKING_AREA(navigation_thd_wa, 256);
THD_FUNCTION(navigation_thd,arg) {

	chRegSetThreadName(__FUNCTION__);
	(void) arg;
	int16_t diffspeed = 0;

	while(1) {

		//Mode suivit de route
		while(get_general_state()==0) {
			clignoter = 0;
			const int8_t Kp = 1;
			int16_t proxLeft = get_prox(7) + get_prox(6);
			int16_t proxRight = get_prox(0) + get_prox(1);
			int16_t diff = proxLeft - proxRight;
			diffspeed = diff*Kp;
			left_motor_set_speed(400 + diffspeed);
			right_motor_set_speed(400 - diffspeed);
			if(diffspeed > 50){
				clignoter = 1;
			}else if(diffspeed < -50){
				clignoter = 2;
			}
			chThdSleepMilliseconds(100);
		}
		clignoter = BLINK_OFF;
		set_rgb_led(LED4, 99,0,0);
		set_rgb_led(LED6, 99,0,0);
		set_led(LED5,2);

		systime_t time;
		int16_t erreur_distance_i = 0;

		//Mode alignement avec le feu rouge
		while(get_general_state()==1) {
			time = chVTGetSystemTime();
			int16_t erreur = get_traffic_light_center() - CENTRE_IMAGE;
			int16_t erreur_distance = CIBLE_TAILLE - get_traffic_light_size();
			erreur_distance_i += erreur_distance*MS2ST(10);
			left_motor_set_speed((erreur_distance*3 /*+ erreur_distance_i/300*/) + KP_FEU*erreur/2);
			right_motor_set_speed((erreur_distance*3 /*+ erreur_distance_i/300*/) - KP_FEU*erreur/2);
			chThdSleepUntilWindowed(time, time + MS2ST(10));
		}
		set_rgb_led(LED4, 0,0,0);
		set_rgb_led(LED6, 0,0,0);
		set_led(LED5,0);
	}
}

void navigation_start(void) {
	chThdCreateStatic(navigation_thd_wa, sizeof(navigation_thd_wa), NORMALPRIO, navigation_thd, NULL);
}

