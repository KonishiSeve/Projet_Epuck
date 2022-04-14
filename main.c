#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
#include <camera/po8030.h>
#include <chprintf.h>

#include <pi_regulator.h>
#include <process_image.h>
#include "ch.h"
#include "hal.h"
#include <sensors/proximity.h>
#include <leds.h>
#include "spi_comm.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

void SendUint8ToComputer(uint8_t* data, uint16_t size) 
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

int main(void)
{
	halInit();
	chSysInit();
	mpu_init();

	serial_start();
	usb_start();

	spi_comm_start();

	dcmi_start();
	po8030_start();
	motors_init();
    messagebus_init(&bus, &bus_lock, &bus_condvar);

	proximity_start();

	messagebus_topic_t *proximity_topic = messagebus_find_topic_blocking(&bus, "/proximity");
	proximity_msg_t prox;
	//pi_regulator_start();
	process_image_start();
	chThdSleepMilliseconds(100);
	calibrate_ir();
	/*set_led(1,1);
	set_body_led(1);
	set_rgb_led(LED4, 99, 0, 0);
	set_rgb_led(LED6, 0,99,0);
	set_rgb_led(LED8,0,0,30);*/

	//int16_t speedi = 200;
	//int16_t speedi2 = 200;
	int8_t Kp = 1;
	int16_t speedk = 200;
	int16_t speedk2 = 200;
	while(1){
		chThdSleepMilliseconds(200);
		/*chprintf((BaseSequentialStream *)&SDU1, "calib = %x \n", get_calibrated_prox(1));
		//chprintf((BaseSequentialStream *)&SDU1, "prox = %x \n", get_prox(1));
		chprintf((BaseSequentialStream *)&SDU1, "diff = %x \n", get_ambient_light(0));
		chprintf((BaseSequentialStream *)&SDU1, "amb1 = %x \n", get_ambient_light(1));
		chprintf((BaseSequentialStream *)&SDU1, "amb2 = %x \n", get_ambient_light(2));
		set_rgb_led(2, 0, 99, 0);*/
		//chprintf((BaseSequentialStream *)&SDU1, "proxright = %d ", get_prox(0));
		//chprintf((BaseSequentialStream *)&SDU1, "proxleft = %d ", get_prox(7));

		int16_t proxLeft = get_prox(7) + get_prox(6);
		int16_t proxRight = get_prox(0) + get_prox(1);

		int16_t diff = proxLeft - proxRight;
		speedk = diff * Kp;
		speedk2 =  - diff * Kp;
		/*if(proxLeft - proxRight > 100){
			if(speedi<1000){
				speedi += 100;
			}
			if(speedi2 > -1000){
				speedi2 -= 100;
			}
		} else if(proxRight - proxLeft > 100){
			if(speedi2<1000){
				speedi2 += 100;
			}
			if(speedi > -1000){
				speedi -= 100;
			}
		} else {
			speedi = 200;
			speed2i = 200;
		}*/
		left_motor_set_speed(200 + speedk);
		right_motor_set_speed(200 + speedk2);
	}


}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
