/*
 * controller.h
 *
 *  Created on: Dec 10, 2017
 *      Author: smate
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "register_map.h"
#include "remotecontrol.h"



void SetServo(float ServoSP);
void SetMotor(int motorSpeed);
void SetRollingInfraServo(int angle);

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

#endif /* CONTROLLER_H_ */
