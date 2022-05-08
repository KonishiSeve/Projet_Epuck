#ifndef CALIBRATION_H_
#define CALIBRATION_H_
//Fichier contenant toutes les constantes qui servent a calibrer l'analyse d'image et les controleurs P/PI de navigation


//detection de pic rouge
#define RED_PEAK_THRESHOLD_DIVIDER 1.3f
#define RED_STD_DIVIDER 100

//detection de feu rouge/vert
#define RED_STD_BOTTOM_THRESHOLD 0
#define RED_STD_TOP_THRESHOLD 10
#define RED_MEAN_THRESHOLD 21
#define RED_TRIGGER_THRESHOLD 3

#define THRESHOLD_GREEN 10

//detection de jour/nuit
#define NIGHT_THRESHOLD 15
#define DAY_THRESHOLD 80

#endif /* CALIBRATION_H_ */
