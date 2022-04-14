#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;
    int16_t error = 0;
    int16_t error_i = 0;
    uint16_t Kp = 180;
    uint16_t Ki = 1;
    while(1){
        time = chVTGetSystemTime();
        error = 10 - get_distance_cm();
        error_i += error * MS2ST(10);
        speed = Kp * error + Ki * error_i/100;
        //chprintf((BaseSequentialStream *)&SDU1, "Integral term = %f \n", Ki * error_i/100);
        //applies the speed from the PI regulator
		 right_motor_set_speed(speed - 50*(get_centre() - 320)/50);
		 left_motor_set_speed(speed + 50*(get_centre() - 320)/50);
		 chprintf((BaseSequentialStream *)&SDU1, "speed = %d \n",(5*(get_centre() - 320)/50));
		 //chprintf((BaseSequentialStream *)&SDU1, "leftspeed = %d \n", get_centre());
		 chprintf((BaseSequentialStream *)&SDU1, "centre = %d \n", get_centre());
        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
