#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"

#include <camera/po8030.h>
#include <chprintf.h>
#include <leds.h>
#include <motors.h>
#include <sensors/proximity.h>

#include <clignotant.h>
#include <main.h>
#include <navigation.h>
#include <process_image.h>

//Initialisation du message bus pour les capteurs de proximite
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

//DELETE Pour le debug
void SendUint8ToComputer(uint8_t* data, uint16_t size)
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

//DELETE Utilis� pour le d�bug
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
	//configuration du microcontrolleur
	halInit();
	chSysInit();
	mpu_init();

	serial_start(); //DELETE debug

	//demarrage du SPI pour les leds RGB
	spi_comm_start();

	//configuration de la camera
	dcmi_start();
	po8030_start();

	//configuration des moteurs
	motors_init();
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    //initialisation des capteurs de proximite
	proximity_start();
	messagebus_topic_t *proximity_topic = messagebus_find_topic_blocking(&bus, "/proximity");
	proximity_msg_t prox;

	//Initialisation des thread
	navigation_start();
	blinker_start();
	process_image_start();

	chThdSleepMilliseconds(100);

	while(1){
		//chThdSleepMilliseconds(10); //Les LEDS marchent plus si on le met
	}
}

//Protection du stack
#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;
void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
