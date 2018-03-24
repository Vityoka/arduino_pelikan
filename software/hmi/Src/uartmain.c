/*
 * uartmain.cpp
 *
 *  Created on: 2017. nov. 10.
 *      Author: Vityó
 */

#include "uartmain.h"
#include "stm32f0xx_hal.h"
#include "string.h"
#include "inttypes.h"


#define MAX_RX_BUF 8
#define INSTRUCTION_LENGTH 5

uint8_t Rx_buf [MAX_RX_BUF];

extern UART_HandleTypeDef huart1;

void uart_startReceiving() {
	__HAL_UART_FLUSH_DRREGISTER(&huart1);
	HAL_UART_Receive_IT(&huart1 , &Rx_buf , INSTRUCTION_LENGTH);
}

void uart_send( uint8_t* data , uint8_t size ) {
	HAL_UART_Transmit(&huart1 , (uint8_t*)&data , size , 200);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	process_uart_msg();
	HAL_UART_Receive_IT(&huart1 , &Rx_buf , INSTRUCTION_LENGTH);
}

inline void process_uart_msg()		//beérkezõ parancsok feldolgozása
{
	if( strcmp((char*)Rx_buf , "dbgof" ) == 0)
	{

	}
	else if( strcmp((char*)Rx_buf , "dbgon") == 0)
	{

	}
	else if( strcmp((char*)Rx_buf , "dataa") == 0)
	{

	}
	else if( strcmp((char*)Rx_buf , "calwh") == 0)
	{

	}
	else if( strcmp((char*)Rx_buf , "calbl") == 0)
	{

	}
}

