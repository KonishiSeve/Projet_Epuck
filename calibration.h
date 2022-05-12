#ifndef CALIBRATION_H_
#define CALIBRATION_H_
//Fichier contenant toutes les constantes qui servent a calibrer l'analyse d'image et les controleurs P/PI de navigation


//detection de pic rouge
#define RED_SLOPE_SHARPNESS 3

//detection de feu rouge
#define RED_MEAN_THRESHOLD 12
#define RED_PEAK_TRIGGER 3
#define RED_PEAK_WIDTH_THRESHOLD 70
#define RED_STD_THRESHOLD_LOW 1
#define RED_STD_THRESHOLD_HIGH 120

//detection de feu vert
#define GREEN_MEAN_THRESHOLD 60

//detection de jour/nuit
#define NIGHT_THRESHOLD 8
#define NIGHT_TRIGGER_THRESHOLD 50

//Capteurs de proximite
#define PROX_COEFF_17DEG 10
#define PROX_COEFF_49DEG 1

#endif /* CALIBRATION_H_ */
