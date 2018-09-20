/*
 * buttons.c
 *
 *  Created on: 2018. márc. 30.
 *      Author: Vityó
 */

#include "buttons.h"
#include "uartmain.h"
#include "ssd1306.h"

uint8_t ButtonsDebounced;		// Debounced state of the buttons
uint8_t ButtonsPressed;			// Rising edge for user input
uint8_t ButtonsRaw[MAX_CHECKS];	// Array that maintains bounce status
uint8_t Index = 0;				// Current Index of Buttons array

//Read inputs and debounce buttons
void DebounceButtons()
{
	//Local variables
    uint8_t i, temp;

    //Read button inputs
    ButtonsRaw[Index] = 0x00;
    ButtonsRaw[Index] |= (HAL_GPIO_ReadPin(MENU_UP_GPIO_Port, MENU_UP_Pin) << 6);
    ButtonsRaw[Index] |= (HAL_GPIO_ReadPin(MENU_DOWN_GPIO_Port, MENU_DOWN_Pin) << 5);
    ButtonsRaw[Index] |= (HAL_GPIO_ReadPin(MENU_LEFT_GPIO_Port, MENU_LEFT_Pin) << 4);
    ButtonsRaw[Index] |= (HAL_GPIO_ReadPin(MENU_RIGHT_GPIO_Port, MENU_RIGHT_Pin) << 3);
    ButtonsRaw[Index] |= (HAL_GPIO_ReadPin(MENU_1_GPIO_Port, MENU_1_Pin) << 2);
    ButtonsRaw[Index] |= (HAL_GPIO_ReadPin(MENU_2_GPIO_Port, MENU_2_Pin) << 1);
    ButtonsRaw[Index] |= (HAL_GPIO_ReadPin(MENU_3_GPIO_Port, MENU_3_Pin) << 0);

    //Increment index
    ++Index;
    if(Index>=MAX_CHECKS)
    	Index=0;

    //Debounce magic
    temp=0xff;
    for (i=0; i<MAX_CHECKS-1;i++)
    	temp=temp & ButtonsRaw[i];
    ButtonsPressed |= ((~ButtonsDebounced) & temp);
    ButtonsDebounced = temp;
}

void ButtonHandler(){
	if(ButtonsPressed & MENU_UP)
	{
		MenuLogic(4);
		//MenuUp();
	}

	if(ButtonsPressed & MENU_DOWN)
	{
		MenuLogic(5);
		//MenuDown();
	}
	if(ButtonsPressed & MENU_LEFT)
		  MenuPrev();
	if(ButtonsPressed & MENU_RIGHT)
		  MenuNext();
	if(ButtonsPressed & MENU_1)
	{
		MenuLogic(1);
	}
	if(ButtonsPressed & MENU_2)
	{
		MenuLogic(2);
	}
	if(ButtonsPressed & MENU_3)
	{
		MenuLogic(3);
	}
	ButtonsPressed=0x00;
}



