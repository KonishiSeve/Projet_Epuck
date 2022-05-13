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

//utilise pour communiquer avec le thread clignotant
static uint8_t clignoter = BLINK_OFF;
uint8_t get_cligno(){
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
			//CHANGE ?
			chThdSleepMilliseconds(100);
		}
		clignoter = BLINK_OFF;
		//CHANGE ? before 200
		chThdSleepMilliseconds(100);

		//Allumage des phares de freinage
		set_rgb_led(LED4, COLOR_RED);
		set_rgb_led(LED6, COLOR_RED);
		set_led(LED5,2);

		systime_t time;
		int16_t distance_error_i = 0;
		int16_t distance_error_d;
		int16_t distance_error_p = TARGET_SIZE - get_traffic_light_size(); //DELETE if no filtering
		int16_t distance_last_error = distance_error_p;

		//Mode alignement avec le feu rouge
		while(get_general_state() == STATE_TRAFFIC_LIGHT) {
			time = chVTGetSystemTime();
			int16_t rotation_error = get_traffic_light_center() - IMAGE_CENTER;

			//Filtrage de valeurs fausses (trop grandes par rapport aux autres)
			if(abs(distance_error_p) + DISTANCE_FILTER_THRESHOLD > abs(TARGET_SIZE - get_traffic_light_size())) {
				distance_error_p = TARGET_SIZE - get_traffic_light_size();
				distance_error_i += distance_error_p;
				distance_error_d = (distance_error_p - distance_last_error);
				distance_last_error = distance_error_p;
			}
			else {
				//DELETE debug
				//chprintf((BaseSequentialStream *)&SD3, " ===== FILTERED ===== value: %d , actual: %d \r\n",TARGET_SIZE - get_traffic_light_size(), distance_error_p);
			}
			left_motor_set_speed(DISTANCE_KP*distance_error_p + DISTANCE_KI*distance_error_i + DISTANCE_KD*distance_error_d + ROTATION_KP*rotation_error);
			right_motor_set_speed(DISTANCE_KP*distance_error_p + DISTANCE_KI*distance_error_i + DISTANCE_KD*distance_error_d - ROTATION_KP * rotation_error);

			//DELETE debug
			/*
			chprintf((BaseSequentialStream *)&SD3, "Size: %d", get_traffic_light_size());
			chprintf((BaseSequentialStream *)&SD3, " , Center: %d", get_traffic_light_center());
			chprintf((BaseSequentialStream *)&SD3, " , Prop: %d", DISTANCE_KP*distance_error_p);
			chprintf((BaseSequentialStream *)&SD3, " , Integ: %f", distance_error_i*DISTANCE_KI);
			chprintf((BaseSequentialStream *)&SD3, " , Deriv: %d \r \n", DISTANCE_KD*distance_error_d);*/
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

