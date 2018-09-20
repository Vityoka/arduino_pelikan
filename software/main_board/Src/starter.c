/*
 * starter.c
 *
 *  Created on: Nov 7, 2017
 *      Author: smate
 */

#include "starter.h"
#include "register_map.h"
#include "statemachine.h"

extern EventGroupHandle_t Eventgroup_Triggers;

volatile uint8_t starter_rx[10];
volatile uint8_t starter_tx[10];
volatile uint8_t RX_buf;

void StarterUartRxCallback()
{
	if( StarterBegin == 0)
	{
		StarterBegin = 1;
	}
	if( RX_buf == '0')
	{
		StarterReady = 1;
	}
}

void StarterEXTICallback(){
	if( Mode == GYORSASAGI)
	{
		Running = RUN_STOP;
		SpeedSP = 0;
	}
	else if( Mode == UGYESSEGI )
	{
		static int num_exti_int = 0;
		num_exti_int++;
		if( num_exti_int == 1 )
		{
			StarterBegin = 1;
			xEventGroupSetBitsFromISR(Eventgroup_Triggers , BIT_STARTER , pdFALSE );
		}
		if( num_exti_int == 18)
		{
			StarterReady = 1;
			num_exti_int = 0;
		}
	}
}

void StarterInit(void)
{
	/*
	int hibakod = HAL_UART_Receive_IT(&huart2, &RX_buf, 1 );
	if( hibakod != HAL_OK)
	{
		int hiba = 1;
	}
	*/
	return;
}

void StarterFlushRX(void)
{
	unsigned short i=0;
	rx_pointer = 0;
	for(i=0; i<10; i++)
	{
		starter_rx[i] = 0;
	}
	return;
}

void StarterFlushTX(void)
{
	unsigned short i=0;
	for(i=0; i<10; i++)
	{
		starter_tx[i] = 0;
	}
	return;
}


