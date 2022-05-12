#ifndef CALIBRATION_H_
#define CALIBRATION_H_
//Fichier contenant toutes les constantes qui servent a calibrer l'analyse d'image et les controleurs P/PI de navigation


//detection de pic rouge
#define RED_SLOPE_SHARPNESS 3

//detection de feu rouge
#define RED_MEAN_THRESHOLD 12
#define RED_PEAK_TRIGGER 3
#define RED_PEAK_WIDTH_THRESHOLD 70
#define RED_STD_THRESHOLD_LOW 2
#define RED_STD_THRESHOLD_HIGH 30

//detection de feu vert
#define GREEN_MEAN_THRESHOLD 60

//detection de jour/nuit
#define NIGHT_THRESHOLD 15
#define NIGHT_TRIGGER_THRESHOLD 20

#endif /* CALIBRATION_H_ */
