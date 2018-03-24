#ifndef LINE_SENSOR_H_
#define LINE_SENSOR_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "register_map.h"
#include "main.h"
#include "math.h"
#include "stdlib.h"
#include "controller.h"

#define LINENUMCNTR_DIFF 5
#define REGION_DIFF 2
#define SAMPLING_DISTANCE 0.6

enum linetype {decelerate, accelerate };



uint8_t contaigous_regions_back;
uint8_t contaigous_regions_back_prev;
uint8_t v;
enum linetype sensed_linetype;

void LineInit(SPI_HandleTypeDef * SPI_Handle); //initialize the line sensor
void LineFlushTX(void);
void LineFlushRX(void);
void LineSortData(void);
void LineSend(uint8_t size);			//send command over SPI
void LineSendReceive(uint8_t size);
void TaskLineCalc(void); 		//calculates position and oriantation from raw data
void LinetypeHypotheserGyorsasagi();
void LinetypeHypotheserUgyessegi();
uint8_t UthosszCounter( float distanceTarget );
void LineChangeCounter();
void CheckLineNumArray();

SPI_HandleTypeDef * LineSPIHandle;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;

#endif /* LINE_SENSOR_H_ */
