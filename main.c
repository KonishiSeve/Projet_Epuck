#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"

#include <camera/po8030.h>
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


int main(void)
{
	//configuration du microcontrolleur
	halInit();
	chSysInit();
	mpu_init();

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
		//si on met un sleep ici, les LEDS RGB marchent plus
	}
}

//Protection du stack
#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;
void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
