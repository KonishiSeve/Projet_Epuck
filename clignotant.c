#include "ch.h"
#include "hal.h"
#include <leds.h>
#include <main.h>
#include <navigation.h>

#define ORANGE 35,10,0
#define ETEINT 0,0,0
//#define COULEUR_CLIGNO 99,73,12
//#define ETEINDRE_CLIGNO 0,0,0

// thread qui gère le clignotant
//sleep 250ms
//recoit de navigation s'il faut clignoter

THD_WORKING_AREA(clignotant_thd_wa, 256);
THD_FUNCTION(clignotant_thd,arg) {

	chRegSetThreadName(__FUNCTION__);
	(void) arg;

	while(1) {
		uint8_t clignoter = get_cligno();
		if (clignoter==1) {
			set_rgb_led(LED2, ORANGE);
			set_rgb_led(LED4, ORANGE);
			chThdSleepMilliseconds(250);
			set_rgb_led(LED2, ETEINT);
			set_rgb_led(LED4, ETEINT);
			chThdSleepMilliseconds(250);
		}else if(clignoter==2){
			set_rgb_led(LED6, ORANGE);
			set_rgb_led(LED8, ORANGE);
			chThdSleepMilliseconds(250);
			set_rgb_led(LED6, ETEINT);
			set_rgb_led(LED8, ETEINT);
			chThdSleepMilliseconds(250);
		}else{
		chThdSleepMilliseconds(100);
		}
	}

}
void clignotant_start(void){
	chThdCreateStatic(clignotant_thd_wa, sizeof(clignotant_thd_wa), NORMALPRIO, clignotant_thd, NULL);
};
