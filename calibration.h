#ifndef CALIBRATION_H_
#define CALIBRATION_H_
//Fichier contenant toutes les constantes qui servent a calibrer l'analyse d'image et les controleurs P/PI de navigation


//detection de pic rouge
#define RED_PEAK_RED_THRESHOLD_COEFF 0.7f
#define RED_PEAK_GREEN_THRESHOLD_COEFF 1.2f
#define RED_PEAK_BLUE_THRESHOLD_COEFF 1.2f

//detection de pic vert
#define GREEN_PEAK_RED_THRESHOLD_COEFF 1.2f
#define GREEN_PEAK_GREEN_THRESHOLD_COEFF 0.7f
#define GREEN_PEAK_BLUE_THRESHOLD_COEFF 1.2f

//detection de feu rouge
#define RED_PEAK_TRIGGER 3
#define RED_PEAK_WIDTH_THRESHOLD 100

//detection de feu vert
#define GREEN_PEAK_TRIGGER 3
#define GREEN_PEAK_WIDTH_THRESHOLD 100

//detection de jour/nuit
#define NIGHT_THRESHOLD 15
#define DAY_THRESHOLD 80

#endif /* CALIBRATION_H_ */
