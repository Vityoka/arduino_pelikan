/*
 * buttons.h
 *
 *  Created on: 2018. márc. 30.
 *      Author: Vityó
 */

#ifndef BUTTONS_H_
#define BUTTONS_H_

#include "main.h"
#include "inttypes.h"
#include "stm32f0xx_hal.h"
#include "menu.h"

/* Private variables ---------------------------------------------------------*/
#define MAX_CHECKS 10			// # checks before a button is debounced

//[x | MENU_UP | MENU_DOWN | MENU_LEFT | MENU_RIGHT | MENU_1 | MENU_2 | MENU_3]
#define MENU_UP		0b01000000
#define MENU_DOWN	0b00100000
#define MENU_LEFT	0b00010000
#define MENU_RIGHT	0b00001000
#define MENU_1		0b00000100
#define MENU_2		0b00000010
#define MENU_3		0b00000001


void ButtonHandler();
void DebounceButtons();

#endif /* BUTTONS_H_ */
