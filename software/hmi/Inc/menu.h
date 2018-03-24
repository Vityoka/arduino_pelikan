#include "stm32f0xx_hal.h"

#ifndef MENU_H_
#define MENU_H_

//Scroll Right
void MenuNext(void);

//Scroll Left
void MenuPrev(void);

//Visualize Current Menu
void MenuDraw(uint8_t MenuCurrent);


#endif /* MENU_H_ */
