/*
 * line_sensor.c
 *
 *  Created on: Nov 7, 2017
 *      Author: smate
 */


#include "line_sensor.h"
#include "string.h"
#include "cmsis_os.h"
#include "mems.h"
#include "starter.h"
#include "statemachine.h"
#include "bluetooth.h"
#include "controller.h"
#include "stdlib.h"

extern SemaphoreHandle_t Semaphore_linesensor_ready;
extern SemaphoreHandle_t Semaphore_SPI_vonal_buffer;
extern SemaphoreHandle_t Semaphore_Uthossz_Achieved;
extern EventGroupHandle_t Eventgroup_Triggers;

#define LineNumArraySize 12
#define ActiveInfraNumArraySize 20

int LineNumChg = 0;
int LineNumMax = 0;
int Num0Lines = 0;
int Num1Lines = 0;
int Num2Lines = 0;
int Num3Lines = 0;
int ActiveInfraMax = 0;

const int ActiveInfraNumThreshold = 10;
uint8_t LineNumArray[LineNumArraySize];
uint8_t ActiveInfraNumArray[ActiveInfraNumArraySize];
uint8_t LineNumFront_Prev;

void LineDetectorTask(void const * argument)
{
	float UthosszPrev = 0;
	float UthosszPrevInfraNum = 0;
	int LineNumIndex = 0;
	int ActiveInfraNumIndex = 0;
	int MerolegesCntr = 0;
	uint8_t cntr = 0;

	for(int i = 0 ; i < LineNumArraySize ; i++)
	{
		LineNumArray[i] = 0;
	}
	while(1)
	{
		if( (Uthossz - UthosszPrev ) > 0.04 )
		{
			LineNumArray[LineNumIndex] = LineNumFront;
			LineNumIndex++;
			if( LineNumIndex == LineNumArraySize )
				LineNumIndex = 0;
			UthosszPrev = Uthossz;
		}

		if( ( Uthossz - UthosszPrevInfraNum ) > 0.005 )
		{
			ActiveInfraNumArray[ActiveInfraNumIndex] = ActiveInfraFront;
			ActiveInfraNumIndex++;
			if( ActiveInfraNumIndex == ActiveInfraNumArraySize )
				ActiveInfraNumIndex = 0;
			UthosszPrevInfraNum = Uthossz;
		}

		if( ActiveInfraNumArray[(ActiveInfraNumIndex + 1) % ActiveInfraNumArraySize] > ActiveInfraNumThreshold )
		{
			cntr = 0;
			MerolegesCntr = 0;
			for( int i = 0 ;  i < ActiveInfraNumArraySize ; i++)
			{
				if( ActiveInfraNumArray[i] > ActiveInfraNumThreshold)
				{
					cntr++;
				}
				else
				{
					if( cntr != 0 )
					{
						MerolegesCntr++;
						cntr = 0;
					}
					else
					{
						cntr = 0;
					}
				}
			}
			if(cntr != 0)
			{
				MerolegesCntr++;
				cntr = 0;
			}
			if(MerolegesCntr == 1)
			{
				xEventGroupSetBits(Eventgroup_Triggers , BIT_CEL );
			}
			else if( MerolegesCntr == 2 )
			{
				xEventGroupSetBits(Eventgroup_Triggers , BIT_VASUTIATJARO );
			}
			for( int i = 0; i < ActiveInfraNumArraySize ;  i++ )
			{
				ActiveInfraNumArray[i] = 0;
			}
		}

		osDelay(10);
	}
	osThreadTerminate(NULL);
}

void DetectTask(void const * argument)
{
	while(1)
	{
		if( Mode == GYORSASAGI)
		{
			if( Running == RUN_FULL_AUTO )
			{
				LinetypeHypotheserGyorsasagi();
			}
		}
		else
		{
			CheckLineNumArray();
			LinetypeHypotheserUgyessegi();
		}

		osDelay(50);
	}
	osThreadTerminate(NULL);
}

void LineGetData(void const * argument)
{
	HAL_GPIO_WritePin(VonalGetData_GPIO_Port , VonalGetData_Pin , GPIO_PIN_SET );
	HAL_GPIO_WritePin(VonalGetData_GPIO_Port , VonalGetData_Pin , GPIO_PIN_RESET );

	while(1)
	{
		if ( xSemaphoreTake( Semaphore_linesensor_ready, 500)  != pdTRUE)	//megvárjuk amíg jelez a vonalszenzor hogy végzett a számítással
		{
			//asm("bkpt");
		}
		else
		{
			LineSendReceive(16);
			if ( xSemaphoreTake( Semaphore_SPI_vonal_buffer, 500)  != pdTRUE)	//az írási buffert lefoglaljuk
			{
				//asm("bkpt");
			}
			HAL_GPIO_WritePin(VonalGetData_GPIO_Port , VonalGetData_Pin , GPIO_PIN_SET );
			//for( int i = 0 ; i < 10000 ; i++); //kell delay mert a main sokkal gyorsabb mint a vonalszenzor, és különben nem venné be a pulzust.
			//TODO, ez a késleltetsé túl nagy, de kisebbre valamiért kifagy.
			HAL_Delay(1);
			HAL_GPIO_WritePin(VonalGetData_GPIO_Port , VonalGetData_Pin , GPIO_PIN_RESET );
		}
	}
	osThreadTerminate(NULL);
}

void LineInit( SPI_HandleTypeDef* SPI_Handle)
{
	LineSPIHandle = SPI_Handle;
	LineFlushTX();
	LineFlushRX();
	LineDataWait = 0x00;
	LineDataReady = 0x00;
}

void LineFlushTX(void)
{
	unsigned short i=0;
	for(i=0; i<LINE_BUFF_TX_SIZE; i++)
	{
		line_buffer_tx[i] = 0;
	}
	return;
}

void LineFlushRX(void)
{
	unsigned short i=0;
	for(i=0; i<LINE_BUFF_RX_SIZE; i++)
	{
		line_buffer_rx[i] = 0;
	}
	return;
}

void LineSend(uint8_t size)
{
	unsigned short i=0;
	while(line_buffer_tx[i] != 0)
		i++;
	HAL_GPIO_WritePin(Vonal_NSS_GPIO_Port, Vonal_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_DMA(LineSPIHandle, line_buffer_tx, size);
}

void LineSendReceive(uint8_t size)
{
	/*
	unsigned short i=0;
	while(line_buffer_tx[i] != 0)
		i++;
	*/
	HAL_GPIO_WritePin(Vonal_NSS_GPIO_Port, Vonal_NSS_Pin, GPIO_PIN_RESET);
	//HAL_SPI_Transmit_DMA(LineSPIHandle, line_buffer_tx, i);
	HAL_SPI_TransmitReceive_DMA(LineSPIHandle, line_buffer_tx, line_buffer_rx, size);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi )
{
	if(hspi->Instance == SPI3)
	{
		HAL_GPIO_WritePin(Vonal_NSS_GPIO_Port, Vonal_NSS_Pin, GPIO_PIN_SET);
		LineFlushTX();

	}
	if(hspi->Instance == SPI2)
	{
		HAL_GPIO_WritePin(Vonal_NSS_GPIO_Port, Vonal_NSS_Pin, GPIO_PIN_SET);
	}
}

void LineSortData(void)
{

	memcpy( &LinePositionFront, (line_buffer_rx ), 4);
	LinePositionFront *= 0.006858;			//vonalpozíció átváltása tcrtrõl méterre
	memcpy( &LinePositionBack, (line_buffer_rx + 4), 4);
	LinePositionBack *= 0.006858;			//vonalpozíció átváltása tcrtrõl méterre
	memcpy( &LineAngle, line_buffer_rx + 8, 4);
	memcpy( &LineNumFront, line_buffer_rx + 12, 1);
	memcpy( &LineNumBack, line_buffer_rx + 13, 1);
	memcpy( &ActiveInfraFront, line_buffer_rx + 14, 1);
	memcpy( &ActiveInfraBack, line_buffer_rx + 15, 1);

	LineDataWait = 0x00;
}


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi )
{
	if(hspi->Instance == SPI3)
	{
		HAL_GPIO_WritePin(MEMS_NSS_GPIO_Port, MEMS_NSS_Pin, GPIO_PIN_SET);
		LineFlushTX();
		LineSortData();
		LineFlushRX();
		int retval = xSemaphoreGiveFromISR( Semaphore_SPI_vonal_buffer , NULL );
		if( retval != pdTRUE )
		{
			//asm("bkpt");
		}
	}
	if(hspi->Instance == SPI2)
	{
		MemsTXRXCallback();
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_10)
	{
		int retval = xSemaphoreGiveFromISR( Semaphore_linesensor_ready , NULL );
		if ( retval != pdTRUE )
		{
			//asm("bkpt");
		}
	}
	if(GPIO_Pin == GPIO_PIN_2)
	{
		StarterEXTICallback();
	}
	if(GPIO_Pin == GPIO_PIN_13)
	{
		BluetoothResetCallback();
	}
}

void LinetypeHypotheserGyorsasagi()
{
	if( (LineNumBack == 3) )
	{
		if(GyorsasagiState == 0)
		{
			GyorsasagiState = 1;
			SpeedSP = SpeedSPKanyar;
		}
		else
		{
			GyorsasagiState = 0;
			SpeedSP = SpeedSPGyors;
		}
		osDelay(3000);
	}
}

void CheckLineNumArray()
{
	LineNumChg = 0;
	LineNumMax = 0;
	Num0Lines = 0;
	Num1Lines = 0;
	Num2Lines = 0;
	Num3Lines = 0;

	//legnagyobb darabszámú vonal
	for( int i = 0 ;  i < LineNumArraySize ; i++)
	{
		if( LineNumArray[i] > LineNumMax)
		{
			LineNumMax = LineNumArray[i];
		}
		if( LineNumArray[i] == 0)
		{
			Num0Lines++;
		}
		if( LineNumArray[i] == 1)
		{
			Num1Lines++;
		}
		if( LineNumArray[i] == 2)
		{
			Num2Lines++;
		}
		if( LineNumArray[i] == 3)
		{
			Num3Lines++;
		}
	}

	//vonaldarab változások száma
	for( int i = 0 ;  i < LineNumArraySize-1 ; i++)
	{
		if( LineNumArray[i+1] != LineNumArray[i] )
		{
			LineNumChg++;
		}
	}
	if( LineNumArray[0] != LineNumArray[LineNumArraySize-1])
	{
		LineNumChg++;
	}


}

void LinetypeHypotheserUgyessegi(void)
{
	if( (Num0Lines >= LineNumArraySize*0.35) && (Num1Lines >= LineNumArraySize*0.35) && LineNumChg >= 4 )
	{
		xEventGroupSetBits(Eventgroup_Triggers , BIT_KORFORGALOM );
	}
	if( (Num2Lines >= LineNumArraySize*0.20)  && (Num1Lines >= LineNumArraySize*0.50) && LineNumChg >= 3 )
	{
		xEventGroupSetBits(Eventgroup_Triggers , BIT_FORGOHORDO );
	}
	if( (Num3Lines > LineNumArraySize*0.85) )
	{
		xEventGroupSetBits(Eventgroup_Triggers , BIT_DRONE );
	}
}


void HAL_TIM_PeriodElapsedCallback( TIM_HandleTypeDef *htim )
{
	if(htim->Instance == TIM7)
	{

	}
	else if (htim->Instance == TIM14)
	{
		HAL_IncTick();
	}
	else if(htim->Instance == TIM6)
	{

	}

}
