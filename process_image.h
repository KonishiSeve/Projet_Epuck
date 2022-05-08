#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

void process_image_start(void);

//communication avec navigation
#define STATE_ROAD 0
#define STATE_TRAFFIC_LIGHT 1

uint16_t get_traffic_light_center(void);
uint16_t get_traffic_light_size(void);
uint8_t get_general_state(void);

#endif /* PROCESS_IMAGE_H */
