#ifndef NAVIGATION_H_
#define NAVIGATION_H_

void navigation_start(void);

//communication avec clignotant
#define BLINK_OFF 0
#define BLINK_RIGHT 1
#define BLINK_LEFT 2
uint8_t get_blinker(void);

#endif /* NAVIGATION_H_ */
