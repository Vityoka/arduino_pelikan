/*
 * batteries.h
 *
 *  Created on: Nov 7, 2017
 *      Author: smate
 */

#ifndef ADC_H_
#define ADC_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern SemaphoreHandle_t Semaphore_ADC_batteries;
extern SemaphoreHandle_t Semaphore_ADC_dist_sensors;
extern SemaphoreHandle_t Semaphore_ADC_motor_curr;

#define UtcasarokFalTavolsag 28.0
#define KonvojFalTavolsag  30.0
#define LeftSensorFromCenter  11
#define RightSensorFromCenter  11

float InterpolateDistance(uint16_t MeasuredVoltage , uint8_t sensortype);
void MovingAverage();
void ObstacleHypotheser();

void CheckBatteriesTask(void const * argument);
void ReadDistanceSensorsTask(void const * argument);
void ReadMotorCurrentTask(void const * argument);


#endif /* ADC_H_ */
