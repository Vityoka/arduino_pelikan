/*
 * hmi.c
 *
 *  Created on: Nov 7, 2017
 *      Author: smate
 */

#include "register_map.h"
#include "hmi.h"
#include "bluetooth.h"

extern UART_HandleTypeDef huart4;
extern osThreadId defaultTaskHandle;


uint16_t hmi_rx_index = 0;
volatile uint8_t RX_buf;
volatile uint8_t hmi_buffer_rx[HMI_BUFF_RX_SIZE];
volatile uint8_t hmi_buffer_tx[HMI_BUFF_TX_SIZE];

void HMITask(void)
{
	while(1)
	{
		int size = LoadBuffer(hmi_buffer_tx);
		HMISend(size);
		osDelay(500);
	}
	osThreadTerminate(NULL);
}


void HMIUartRxCallback()
{
	hmi_buffer_rx[hmi_rx_index] = RX_buf;
	hmi_rx_index++;
	HMICommandParser();
	RX_buf = 0;
	HAL_UART_Receive_IT(&huart4, &RX_buf, 1 );
}

void HMISend(uint16_t size)
{

	//HAL_GPIO_TogglePin(GPIOA , GPIO_PIN_0);
	//HAL_GPIO_TogglePin(GPIOA , GPIO_PIN_1);
	int retval = HAL_UART_Transmit_IT(&huart4, hmi_buffer_tx, size);
	if( retval != HAL_OK )
	{
		asm("bkpt");
	}
	return;
}

void HMIReceive(void)
{
	hmi_rx_index = 0;
	HAL_UART_Receive_IT(&huart4, &RX_buf, 1 );
	return;
}

void HMIFlushRX(void)
{
	unsigned short i=0;
	hmi_rx_index = 0;
	for(i=0; i<HMI_BUFF_RX_SIZE; i++)
	{
		hmi_buffer_rx[i] = 0;
	}
	return;
}

void HMIFlushTX(void)
{
	unsigned short i=0;
	for(i=0; i<HMI_BUFF_TX_SIZE; i++)
	{
		hmi_buffer_tx[i] = 0;
	}
	return;
}

inline void HMICommandParser(void)
{
	switch(RX_buf)
	{
	default:
		if( RX_buf == '\n' )
		{
			CommandParser(hmi_buffer_rx , hmi_rx_index);
			HMIFlushRX();
		}
	}
}

