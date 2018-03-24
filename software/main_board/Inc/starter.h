/*
 * starter.h
 *
 *  Created on: Nov 7, 2017
 *      Author: smate
 */

#ifndef STARTER_H_
#define STARTER_H_

#include "register_map.h"

extern UART_HandleTypeDef huart2;

void StarterInit(void);
void StarterUartRxCallback();
void StarterEXTICallback();

#endif /* STARTER_H_ */
