/*
 * SPIMain.cpp
 *
 *  Created on: 2017. nov. 4.
 *      Author: Vityó
 */

#include <SPIMain.h>
#include "stm32f0xx_hal.h"
#include "string.h"


//uint8_t charbuf;
uint8_t Main_rx_buf [MAX_RX_BUF];
uint8_t Main_tx_buf [8] = {0,0,0,0,0,0,0,0};
uint8_t hivas = 0;
volatile uint8_t n = 0;
extern SPI_HandleTypeDef hspi2;
extern  uint8_t tx_type;	//meghatározza az adatküldés típusát, debug mode, vagy csak vonalpozíció
extern  uint8_t data_requested_mainboard_flag;
extern  uint8_t calibrate_white_flag;
extern  uint8_t calibrate_black_flag;

//extern char rx [20];
//extern char tx [20];

namespace V {

	SPIMain::~SPIMain() {
	}

	void SPIMain::send( uint8_t mode ) {
		size_t tx_size = 0;
		infra_ref.serialize( TX_data , &tx_size , mode );
		HAL_SPI_TransmitReceive(&hspi2 , (uint8_t*)TX_data , (uint8_t*)Main_rx_buf , tx_size , 2000 );
		for( int i = 0 ; i < TX_BUFSIZE ; i++)
			TX_data[i] = 0;
	}

	void SPIMain::listen() {
		char tx [8] = {0,0,0,0,0,0,0,0};
		HAL_SPIEx_FlushRxFifo(&hspi2);
		HAL_SPI_TransmitReceive(&hspi2 , (uint8_t*)tx ,  (uint8_t*)Main_rx_buf , INSTRUCTION_LENGTH , 1000 );
		process_commands();
	}

} /* namespace V */

extern "C"
{
	/*
	void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
	{
		if( (hspi->Instance==SPI2) )		//spi2 = mainboard
		{
			//process_commands();
		}
	}
	void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
	{
		if( (hspi->Instance==SPI2) )		//spi2 = mainboard
		{
			//process_commands();
			for( int i = 0 ; i < 20 ; i++)
			{
				tx[i] = rx[i];
			}
		}
	}
	void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
	{
		if( (hspi->Instance==SPI2) )		//spi2 = mainboard
		{
			//process_commands();
			for( int i = 0 ; i < 20 ; i++)
			{
				tx[i] = rx[i];
			}
		}
	}
 	*/
	inline void process_commands()		//beérkezõ parancsok feldolgozása
		{
			if( strcmp((char*)Main_rx_buf , "dbgof" ) == 0)
			{
				tx_type = 0;
			}
			else if( strcmp((char*)Main_rx_buf , "dbgon") == 0)
			{
				tx_type = 1;
			}
			else if( strcmp((char*)Main_rx_buf , "dataa") == 0)
			{
				data_requested_mainboard_flag = 1;
			}
			else if( strcmp((char*)Main_rx_buf , "calwh") == 0)
			{
				calibrate_white_flag = 1;
			}
			else if( strcmp((char*)Main_rx_buf , "calbl") == 0)
			{
				calibrate_black_flag = 1;
			}
		}
	void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
		if(GPIO_Pin == MAIN_WAITDATA_Pin)
		{
			data_requested_mainboard_flag = 1;
		}
	}
}

