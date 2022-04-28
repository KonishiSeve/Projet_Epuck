// thread qui gère le clignotant
//sleep 250ms
//recoit de navigation s'il faut clignoter

while(1) {
	if (clignotant) {
	allumer
	thd_sleep(250)
	eteindre
	thd_sleep(250)
	}
}
