#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>
#include <motors.h>
#include <camera/po8030.h>
#include <chprintf.h>

#include <process_image.h>
#include "ch.h"
#include "hal.h"
#include <sensors/proximity.h>
#include <leds.h>

#include <navigation.h>
#include <clignotant.h>


/* === Liste des threads ===
Nom : fonction, priorité (fichier)

ProcessImage : détecte le feu rouge, NORMALPRIO (process_iamge.c)
CaptureImage : capture une image, NORMALPRIO (process_image.c)

proximity_thd : lit les valeurs des capteurs de proximité, NORMALPRIO, (sensors/proximity.c)


note: Il faudra mesurer le temps d'execution de chaque thread pour définir les priorités correctement

note: les moteurs utilisent directement les timers ??
*/


// Pour faire marcher les capteurs de proximité
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

//Pour le debug
void SendUint8ToComputer(uint8_t* data, uint16_t size)
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

//Utilisé pour le débug
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

	serial_start(); //Pour le débug

	spi_comm_start(); //Pour les leds rgb

	//configuration de la caméra
	dcmi_start();
	po8030_start();

	//configuration des moteurs
	motors_init();
    messagebus_init(&bus, &bus_lock, &bus_condvar); // ????????????????

    //initialisation des capteurs de proximité
	proximity_start();
	messagebus_topic_t *proximity_topic = messagebus_find_topic_blocking(&bus, "/proximity");
	proximity_msg_t prox;
	navigation_start();
	clignotant_start();

	process_image_start();
	chThdSleepMilliseconds(100);
	set_rgb_led(LED4, 99,0,0);
	set_rgb_led(LED6, 99,0,0);

	while(1){
		//chThdSleepMilliseconds(2000);
	}
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
