#include "ch.h"
#include "hal.h"
#include <motors.h>
#include <main.h>
#include <process_image.h>
#include <chprintf.h>
#include <leds.h>
#include <calibration.h>
#include <navigation.h>

#define IMAGE_CENTER 320
#define TARGET_SIZE 130
#define COLOR_RED 99,0,0
#define COLOR_BLACK 0,0,0
#define ROAD_SPEED 400
#define BLINK_THRESHOLD 50
#define DISTANCE_I_CUT_THRESHOLD 500

//utilise pour communiquer avec le thread clignotant
static uint8_t clignoter = BLINK_OFF;
uint8_t get_blinker(){
	return clignoter;
}

//thread qui s'occupe d'eviter les obstacles ou suivre le feu
THD_WORKING_AREA(navigation_thd_wa, 256);
THD_FUNCTION(navigation_thd,arg) {
	chRegSetThreadName(__FUNCTION__);
	(void) arg;
	while(1) {
		// ===== Mode suivit de route =====
		while(get_general_state() == STATE_ROAD) {

			//controleur pour evitement d'obstacle 
			int16_t proxLeft = PROX_COEFF_17DEG*get_prox(7) + PROX_COEFF_49DEG*get_prox(6); //somme ponderee des capteurs a 17 degres gauche et 49 degres gauche
			int16_t proxRight = PROX_COEFF_17DEG*get_prox(0) + PROX_COEFF_49DEG*get_prox(1); //somme ponderee des capteurs a 17 degres droit et 49 degres droit
			int16_t diffspeed = (proxLeft - proxRight) * PROX_KP;
			left_motor_set_speed(ROAD_SPEED + diffspeed);
			right_motor_set_speed(ROAD_SPEED - diffspeed);

			//Activation des clignotants si le virage est serre
			if(diffspeed > BLINK_THRESHOLD){
				clignoter = BLINK_RIGHT;
			}else if(diffspeed < -BLINK_THRESHOLD){
				clignoter = BLINK_LEFT;
			} else {
				clignoter = BLINK_OFF;
			}
			chThdSleepMilliseconds(100);
		}
		clignoter = BLINK_OFF;
		set_rgb_led(LED8, COLOR_BLACK);
		set_rgb_led(LED8, COLOR_BLACK);

		//Allumage des phares de freinage
		set_rgb_led(LED4, COLOR_RED);
		set_rgb_led(LED6, COLOR_RED);
		set_led(LED5,2);

		systime_t time;
		int16_t distance_error_i = 0;
		int16_t distance_error_d;
		int16_t distance_error_p = TARGET_SIZE - get_traffic_light_size();
		int16_t distance_last_error = distance_error_p;

		//Mode alignement avec le feu rouge
		while(get_general_state() == STATE_TRAFFIC_LIGHT) {
			time = chVTGetSystemTime();
			int16_t rotation_error = get_traffic_light_center() - IMAGE_CENTER;

			//Filtrage de valeurs fausses (trop grandes par rapport aux autres)
			if(abs(distance_error_p) + DISTANCE_FILTER_THRESHOLD > abs(TARGET_SIZE - get_traffic_light_size())) {

				//controleur PID
				distance_error_p = TARGET_SIZE - get_traffic_light_size();
				//on coupe le terme integral s'il est trop grand
				if(abs(DISTANCE_KI*distane_error_i) <= DISTANCE_I_CUT_THRESHOLD) {
					distance_error_i += distance_error_p;
				}
				distance_error_d = (distance_error_p - distance_last_error);
				distance_last_error = distance_error_p;
				left_motor_set_speed(DISTANCE_KP*distance_error_p + DISTANCE_KI*distance_error_i + DISTANCE_KD*distance_error_d + ROTATION_KP*rotation_error);
				right_motor_set_speed(DISTANCE_KP*distance_error_p + DISTANCE_KI*distance_error_i + DISTANCE_KD*distance_error_d - ROTATION_KP * rotation_error);
			}
			chThdSleepUntilWindowed(time, time + MS2ST(100));
		}
		set_rgb_led(LED4, COLOR_BLACK);
		set_rgb_led(LED6, COLOR_BLACK);
		set_led(LED5,0);
	}
}

void navigation_start(void) {
	chThdCreateStatic(navigation_thd_wa, sizeof(navigation_thd_wa), NORMALPRIO, navigation_thd, NULL);
}

