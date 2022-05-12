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
#define CIBLE_TAILLE 130
#define KP_FEU 1
#define COLOR_RED 99,0,0
#define COLOR_BLACK 0,0,0

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
			const int8_t Kp = 1;
			int16_t proxLeft = PROX_COEFF_17DEG*get_prox(7) + PROX_COEFF_49DEG*get_prox(6);
			int16_t proxRight = PROX_COEFF_17DEG*get_prox(0) + PROX_COEFF_49DEG*get_prox(1);
			int16_t diff = proxLeft - proxRight;
			diffspeed = diff*Kp;
			left_motor_set_speed(400 + diffspeed);
			right_motor_set_speed(400 - diffspeed);

			if(diffspeed > 50){
				clignoter = BLINK_RIGHT;
			}else if(diffspeed < -50){
				clignoter = BLINK_LEFT;
			} else {
				clignoter = BLINK_OFF;
			}
			chThdSleepMilliseconds(100);
		}
		clignoter = BLINK_OFF;
		chThdSleepMilliseconds(200);
		set_rgb_led(LED4, COLOR_RED);
		set_rgb_led(LED6, COLOR_RED);
		set_led(LED5,2);

		systime_t time;
		int16_t erreur_distance_i = 0;
		int16_t erreur_distance = CIBLE_TAILLE - get_traffic_light_size();

		//Mode alignement avec le feu rouge
		while(get_general_state()==1) {
			time = chVTGetSystemTime();
			int16_t erreur = get_traffic_light_center() - CENTRE_IMAGE;

			/*
			if(abs(erreur_distance) >= abs(CIBLE_TAILLE - get_traffic_light_size())*2) {
				erreur_distance = CIBLE_TAILLE - get_traffic_light_size();
			}
			*/
			erreur_distance = CIBLE_TAILLE - get_traffic_light_size();
			erreur_distance_i += erreur_distance*MS2ST(10);
			left_motor_set_speed((erreur_distance + erreur_distance_i/200) + KP_FEU*erreur/2);
			right_motor_set_speed((erreur_distance + erreur_distance_i/200) - KP_FEU*erreur/2);

			/*
			chprintf((BaseSequentialStream *)&SD3, "Size: %d", get_traffic_light_size());
			chprintf((BaseSequentialStream *)&SD3, " , Center: %d", get_traffic_light_center());
			chprintf((BaseSequentialStream *)&SD3, " , Prop: %d", erreur_distance*2);
			chprintf((BaseSequentialStream *)&SD3, " , Integ: %d \r \n", erreur_distance_i/300);*/
			chThdSleepUntilWindowed(time, time + MS2ST(10));
		}
		set_rgb_led(LED4, COLOR_BLACK);
		set_rgb_led(LED6, COLOR_BLACK);
		set_led(LED5,0);
	}
}

void navigation_start(void) {
	chThdCreateStatic(navigation_thd_wa, sizeof(navigation_thd_wa), NORMALPRIO, navigation_thd, NULL);
}

