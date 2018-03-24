/*
 * ir.h
 *
 *  Created on: Nov 7, 2017
 *      Author: smate
 */

#include "stm32f4xx_hal.h"

#ifndef IR_H_
#define IR_H_

extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim3;
extern EventGroupHandle_t Eventgroup_inputCaptures;

#define ELSO_POZ 1
#define MASODIK_POZ 2
#define HARMADIK_POZ 3
#define ELSO_NEG 4
#define MASODIK_NEG 5
#define HARMADIK_NEG 6


void CalcPulseType();
void InfraReceiverInputCaptureCallback();
void TIM8_ReInit_IC( );
void decode_RC5();
void InfraReceiveTask(void const * argument);
void FlushIRData();

#endif /* IR_H_ */
