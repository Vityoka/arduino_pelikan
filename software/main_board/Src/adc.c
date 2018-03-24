/*
 * batteries.c
 *
 *  Created on: Nov 7, 2017
 *      Author: smate
 */


#include "adc.h"
#include "register_map.h"
#include "stdlib.h"
#include "statemachine.h"

extern EventGroupHandle_t Eventgroup_Triggers;

#define LogicBat_R1 1300
#define LogicBat_R2 4990
#define MotorBat_R1 43000
#define MotorBat_R2 27000
#define ADCunit (3.3/4096.0)

#define MotorCurr_R1 4300
#define MotorCurr_R2 7050

#define LONGRANGE 1
#define SHORTRANGE 0

float DistanceLeftAvg = 0;
float DistanceRightAvg = 0;
float DistanceFrontShortAvg = 0;
float DistanceFrontLongAvg = 0;
volatile float DistanceRightRaw;
volatile float DistanceFrontShortRaw;
volatile float DistanceFrontLongRaw;
volatile float DistanceLeftRaw;
float RightDistances[2];
float LeftDistances[2];

void CheckBatteriesTask(void const * argument){

	while(1)
	{
		uint16_t buf [3];

		HAL_ADC_Start_DMA(&hadc1 ,(uint32_t*)buf , 3);
		if ( xSemaphoreTake( Semaphore_ADC_batteries, 5000)  != pdTRUE)
		{

		}
		BatteryMotor = buf[0] * ADCunit * (((MotorBat_R1 + MotorBat_R2) / ((float)MotorBat_R2)));
		BatteryLogic = buf[1] * ADCunit * (((LogicBat_R1 + LogicBat_R2) / ((float)LogicBat_R2)));
		int nucleo_temp_raw = buf[2];

		osDelay(1000);
	}
	osThreadTerminate(NULL);
}

void ReadDistanceSensorsTask(void const * argument){

	while(1)
	{
		uint16_t buf [4];

		HAL_ADC_Start_DMA(&hadc2 ,(uint32_t*)buf , 4);
		if ( xSemaphoreTake( Semaphore_ADC_dist_sensors, portMAX_DELAY)  != pdTRUE)
		{

		}
		//Be�rkezett nyers adatok interpol�ci�ja
		DistanceRightRaw = InterpolateDistance( buf[0] , SHORTRANGE );
		DistanceFrontShortRaw = InterpolateDistance(  buf[1] , SHORTRANGE );
		DistanceFrontLongRaw = InterpolateDistance( buf[2] , LONGRANGE);
		DistanceLeftRaw = InterpolateDistance( buf[3] , SHORTRANGE );

		//jelek mozg��tlagol�sa
		MovingAverage();

		//Els� k�t szenzor f�zi�ja + Bluetoothon k�ldend� v�ltoz�k friss�t�se
		if( DistanceFrontLongAvg > 30 )
		{
			DistanceFront = DistanceFrontLongAvg;
		}
		else if(DistanceFrontShortAvg <= 30 )
		{
			DistanceFront = DistanceFrontShortAvg;
		}
		else
		{
			DistanceFront = 0;
		}
		DistanceLeft = DistanceLeftAvg;
		DistanceRight = DistanceRightAvg;

		//akad�lydetekt�l�s
		ObstacleHypotheser();

		osDelay(60);
	}
	osThreadTerminate(NULL);
}

void ReadMotorCurrentTask(void const * argument){

	const float VoltageIdle = 2.5;
	const float VoltPerAmp = 0.02;
	while(1)
	{
		uint16_t buf [2];

		HAL_ADC_Start_DMA(&hadc3 ,(uint32_t*)buf , 1);
		if ( xSemaphoreTake( Semaphore_ADC_motor_curr, portMAX_DELAY)  != pdTRUE)
		{

		}
		float MotorCurrentVoltageSensed = buf[0] * ADCunit * (((MotorCurr_R1 + MotorCurr_R2) / ((float)MotorCurr_R2)));
		MotorCurrent = (MotorCurrentVoltageSensed - VoltageIdle) / (float)VoltPerAmp ;

		osDelay(15);
	}
	osThreadTerminate(NULL);
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	if( hadc->Instance == ADC1)
	{
		xSemaphoreGiveFromISR( Semaphore_ADC_batteries, pdFALSE);
	}
	else if( hadc->Instance == ADC2)
	{
		xSemaphoreGiveFromISR( Semaphore_ADC_dist_sensors, pdFALSE);
	}
	else if( hadc->Instance == ADC3)
	{
		xSemaphoreGiveFromISR( Semaphore_ADC_motor_curr, pdFALSE);
	}
}

float InterpolateDistance(uint16_t MeasuredVoltage , uint8_t sensortype){

	#define NUM_OF_DATA 14
	const float SensorCharacteristic[2][NUM_OF_DATA][2] =
	{
		{
			{5   , 2689},
			{7.5 , 1927},
			{10  , 1483},
			{12.5, 1199},
			{15  , 1007},
			{17.5, 873},
			{20  , 760},
			{22.5, 676},
			{25  , 597},
			{27.5, 544},
			{30  , 512},
			{32.5, 476},
			{32.5, 476},
			{32.5, 476}
		},
		{
			{15   , 3410},
			{20   , 3095},
			{25   , 2778},
			{30   , 2427},
			{35   , 2103},
			{40   , 1857},
			{45   , 1647},
			{50   , 1483},
			{60   , 1215},
			{70   , 1063},
			{90   , 809},
			{110  , 638},
			{130  , 467},
			{150  , 267},
		}
	};

	//sz�ls��rt�kek lekezel�se
	if( MeasuredVoltage < SensorCharacteristic[sensortype][NUM_OF_DATA-1][1])
	{
		return SensorCharacteristic[sensortype][NUM_OF_DATA-1][0];
		//return 0;
	}
	if( MeasuredVoltage > SensorCharacteristic[sensortype][0][1])
	{
		return SensorCharacteristic[sensortype][0][0];
		//return 0;
	}

	//index sz�m�t�s
	int index;
	for( index = NUM_OF_DATA-1 ; (MeasuredVoltage > SensorCharacteristic[sensortype][index][1]) && (index > 0); index-- );
	index++;

	//line�ris interpol�ci�
	float distance = (MeasuredVoltage - SensorCharacteristic[sensortype][index][1]) *
					(SensorCharacteristic[sensortype][index-1][0] - SensorCharacteristic[sensortype][index][0]) /
					(SensorCharacteristic[sensortype][index-1][1] - SensorCharacteristic[sensortype][index][1]) +
					SensorCharacteristic[sensortype][index][0];
	return distance;
}

void MovingAverage(){

	#define N 3

	static float PreviousDistancesLeft [N] = {32.5 ,32.5 ,32.5};
	static float PreviousDistancesRight [N]= {32.5 ,32.5 ,32.5};
	static float PreviousDistancesFrontShort [N]= {32.5 ,32.5 ,32.5};
	static float PreviousDistancesFrontLong [N]= {150 ,150 ,150};
	static int index = 0;

	//�j �rt�kek t�rol�sa
	index++;
	if( index == N)
	{
		index = 0;
	}
	PreviousDistancesLeft[index] = DistanceLeftRaw;
	PreviousDistancesRight[index] = DistanceRightRaw;
	PreviousDistancesFrontShort[index] = DistanceFrontShortRaw;
	PreviousDistancesFrontLong[index] = DistanceFrontLongRaw;

	//�tlagol�s
	DistanceLeftAvg = 0;
	for(int i = 0 ; i < N ; i++)
	{
		DistanceLeftAvg += PreviousDistancesLeft[i];
	}
	DistanceLeftAvg /= N;


	DistanceRightAvg = 0;
	for(int i = 0 ; i < N ; i++)
	{
		DistanceRightAvg += PreviousDistancesRight[i];
	}
	DistanceRightAvg /= N;

	if( MovAvgFrontEnabled )
	{
		DistanceFrontShortAvg = 0;
		for(int i = 0 ; i < N ; i++)
		{
			DistanceFrontShortAvg += PreviousDistancesFrontShort[i];
		}
		DistanceFrontShortAvg /= N;

		DistanceFrontLongAvg = 0;
		for(int i = 0 ; i < N ; i++)
		{
			DistanceFrontLongAvg += PreviousDistancesFrontLong[i];
		}
		DistanceFrontLongAvg /= N;
	}
	else
	{
		DistanceFrontShortAvg = DistanceFrontShortRaw;
		DistanceFrontLongAvg = DistanceFrontLongRaw;
	}

}

void ObstacleHypotheser()
{
	//
	LeftDistances[0] = LeftDistances[1];
	LeftDistances[1] = DistanceLeft;

	RightDistances[0] = RightDistances[1];
	RightDistances[1] = DistanceRight;

	if( (LeftDistances[0] < 28.0) && (LeftDistances[1] < 28.0) )	//ha mindk�t mintav�telben volt baloldalon fal
	{
		if( (RightDistances[0] < 28.0) || (RightDistances[1] < 28.0) )	//�s legal�bb az egyik mintav�telben volt jobboldali is akkor utcasarok
		{
			xEventGroupSetBits(Eventgroup_Triggers , BIT_UTCASAROK );
		}
		else	//egy�bk�nt csak baloldalon van fal teh�t konvoj
		{
			KonvojDir = 0;
			xEventGroupSetBits(Eventgroup_Triggers , BIT_KONVOJ );
		}
	}
	if( (RightDistances[0] < 28.0) && (RightDistances[1] < 28.0) )	//ha mindk�t mintav�telben volt jobboldalon fal
	{
		if( (LeftDistances[0] < 28.0) || (LeftDistances[1] < 28.0) )	//�s legal�bb az egyik mintav�telben volt baloldali is akkor utcasarok
		{
			xEventGroupSetBits(Eventgroup_Triggers , BIT_UTCASAROK );
		}
		else	//egy�bk�nt csak jobboldalon van fal teh�t konvoj
		{
			KonvojDir = 1;
			xEventGroupSetBits(Eventgroup_Triggers , BIT_KONVOJ );
		}
	}




	if(DistanceLeftAvg < 25.0 )		//ha mindk�t oldalon fal van utcasarkot tal�ltunk
	{
		if( DistanceRightAvg < 25.0 )
		{
			xEventGroupSetBits(Eventgroup_Triggers , BIT_UTCASAROK );
		}
	}
	if( abs(DistanceLeftAvg - (KonvojFalTavolsag - LeftSensorFromCenter)) < 2.0  )			//ha csak bal oldalon van fal akkor konvojt tal�ltunk
	{
		if( DistanceRightAvg > 50.0 )
		{
			//xEventGroupSetBits(Eventgroup_Triggers , BIT_KONVOJ );
		}
	}


}

