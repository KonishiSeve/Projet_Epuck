#include "ch.h"
#include "hal.h"
#include <leds.h>
#include <main.h>
#include <navigation.h>

#define ORANGE 35,10,0
#define ETEINT 0,0,0
#define BLINK_PERIOD 500

THD_WORKING_AREA(clignotant_thd_wa, 256);
THD_FUNCTION(clignotant_thd,arg) {

	chRegSetThreadName(__FUNCTION__);
	(void) arg;

	while(1) {
		if (get_cligno() == BLINK_RIGHT) {
			set_rgb_led(LED2, ORANGE);
			chThdSleepMilliseconds(BLINK_PERIOD/2);
			set_rgb_led(LED2, ETEINT);
		}
		else if(get_cligno() == BLINK_LEFT) {
			set_rgb_led(LED8, ORANGE);
			chThdSleepMilliseconds(BLINK_PERIOD/2);
			set_rgb_led(LED8, ETEINT);
		}
		chThdSleepMilliseconds(BLINK_PERIOD/2);
	}
}

void clignotant_start(void){
	chThdCreateStatic(clignotant_thd_wa, sizeof(clignotant_thd_wa), NORMALPRIO, clignotant_thd, NULL);
}
