/*
 * uartmain.cpp
 *
 *  Created on: 2017. nov. 10.
 *      Author: Vityó
 */

#include <uartMain.h>
#include "stm32f0xx_hal.h"
#include "string.h"
#include "inttypes.h"
#include "stdlib.h"


#define MAX_RX_BUF 8
#define INSTRUCTION_LENGTH 5

uint8_t Main_rx_buf_uart [MAX_RX_BUF];
char Main_tx_buf_uart [8]= "ABCD";
uint8_t charbuf;
volatile uint8_t m = 0;
extern uint8_t tx_type;	//meghatározza az adatküldés típusát, debug mode, vagy csak vonalpozíció
extern volatile uint8_t data_requested_mainboard_flag;
extern uint8_t calibrate_white_flag;
extern uint8_t calibrate_black_flag;



extern UART_HandleTypeDef huart1;

uartMain::~uartMain() {
	// TODO Auto-generated destructor stub
}

void uartMain::startReceiving() {
	__HAL_UART_FLUSH_DRREGISTER(&huart1);
	//HAL_UART_Receive_DMA(&huart1, (uint8_t*)&charbuf, 1);				//1 karakter hosszú fogadáshoz
	HAL_UART_Receive_DMA(&huart1, (uint8_t*)&Main_rx_buf_uart , INSTRUCTION_LENGTH);	//instruction length hosszú fogadáshoz
}

void uartMain::send( uint8_t mode ) {
	size_t tx_size = 0;
	infra_ref.serialize( TX_data , &tx_size , mode );
	HAL_UART_Transmit(&huart1 , (uint8_t*)&TX_data , tx_size , 200);
}

void uartMain::notify_readiness() {
	char tx = 'R';
	HAL_UART_Transmit(&huart1 , (uint8_t*)&tx , 1 , 200);
}

extern "C"
{
	void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
	{
		//1 karakteres fogadáshoz ezt próbáltam megírni
		/*
		//__HAL_UART_SEND_REQ(&huart1, UART_RXDATA_FLUSH_REQUEST); // Clear the buffer to prevent overrun
		//__HAL_UART_FLUSH_DRREGISTER(&huart1); // Clear the buffer to prevent overrun
		if( m > MAX_RX_BUF)
		{
			m = 0;
			//vmi errorral kéne jelezni
			for(int i = 0 ; i < MAX_RX_BUF ; i++)
			{
				Main_rx_buf_uart[i] = 0;
			}
		}

		if( (charbuf == '\n') || (charbuf == 'i'))	//ne küldjünk egyszerre mindkettõt!
		{
			Main_rx_buf_uart[m] = '\0';
			m = 0;
			process_uart_msg();
			for(int i = 0 ; i < MAX_RX_BUF ; i++)
			{
				Main_rx_buf_uart[i] = 0;
			}
		}
		else
		{
			Main_rx_buf_uart[m] = charbuf;
			m++;
		}
		//__HAL_UART_SEND_REQ(&huart1, UART_RXDATA_FLUSH_REQUEST); // Clear the buffer to prevent overrun
		//__HAL_UART_FLUSH_DRREGISTER(&huart1); // Clear the buffer to prevent overrun
		*/

		//instruction length hosszúságú fogadásra meg ezt írtam meg
		process_uart_msg();
	}

	inline void process_uart_msg()		//beérkezõ parancsok feldolgozása
	{
		if( strcmp((char*)Main_rx_buf_uart , "dbgof" ) == 0)
		{
			tx_type = 0;
		}
		else if( strcmp((char*)Main_rx_buf_uart , "dbgon") == 0)
		{
			tx_type = 1;
		}
		else if( strcmp((char*)Main_rx_buf_uart , "dataa") == 0)
		{
			data_requested_mainboard_flag = 1;
		}
		else if( strcmp((char*)Main_rx_buf_uart , "calwh") == 0)
		{
			calibrate_white_flag = 1;
		}
		else if( strcmp((char*)Main_rx_buf_uart , "calbl") == 0)
		{
			calibrate_black_flag = 1;
		}
	}
}

