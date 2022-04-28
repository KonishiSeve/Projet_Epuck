//Thread pour éviter les obstacles <-> s'aligner avec le feu
// sleep 200ms
//envoie à clignotant
//reçoit de process_image si le feu est vert ou rouge
//reçoit les données des capteurs infrarouge

void start_navigation(void) {
	chThdCreateStatic(navigation_thd_wa, sizeof(navigation_thd_wa), NORMALPRIO, navigation_thd, NULL);
}

THD_WORKING_AREA(navigation_thd_wa, 256);

THD_FUNCTION(navigation_thd,arg) {

	chRegSetThreadName(__FUNCTION__);
	(void) arg;

	while(1) {
		while(pas de feu()) {
			éviter obstacle()
			thd_sleep()
		}
		attend_que_le_feu_soit_vert()
	}
}
