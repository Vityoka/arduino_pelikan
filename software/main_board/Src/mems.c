/*
 * mems.c
 *
 *  Created on: Nov 7, 2017
 *      Author: smate
 */


#include "register_map.h"
#include "mems.h"

#define MEMS_RX_SIZE 2
#define MEMS_TX_SIZE 2
uint8_t MEMS_RX[MEMS_RX_SIZE];
uint8_t MEMS_TX[MEMS_TX_SIZE];

void FlushMemsRx(){
	for(int i = 0; i < MEMS_RX_SIZE ; i++)
	{
		MEMS_RX[i] = 0;
	}
}

void FlushMemsTx(){
	for(int i = 0; i < MEMS_TX_SIZE ; i++)
	{
		MEMS_TX[i] = 0;
	}
}

void MemsReadByte( uint8_t address ){
	FlushMemsRx();
	MEMS_TX[0] = address;
	MEMS_TX[1] = 0;
	HAL_GPIO_WritePin(MEMS_NSS_GPIO_Port, MEMS_NSS_Pin, GPIO_PIN_RESET);
	int retval = HAL_SPI_Transmit(&hspi2, MEMS_TX , 1, 100);
	if( retval != HAL_OK)
	{
		//asm("bkpt 255");
	}
	if( HAL_SPI_Receive(&hspi2, MEMS_RX , 1, 100) != HAL_OK )
	{
		//asm("bkpt 255");
	}
/*
	if( HAL_SPI_TransmitReceive_DMA(&hspi2, MEMS_TX , MEMS_RX, 2) != HAL_OK )
	{
		asm("bkpt 255");
	}
*/
}

void MemsWriteByte( uint8_t address , uint8_t data ){
	FlushMemsTx();
	MEMS_TX[0] = address;
	MEMS_TX[1] = 0;
	HAL_GPIO_WritePin(MEMS_NSS_GPIO_Port, MEMS_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive_DMA(&hspi2, MEMS_TX , MEMS_RX, 2);
}

void MemsWriteBurst( uint8_t address , uint8_t* data , uint8_t size){
	for(int i = 0; i < size ; i++)
	{
		MEMS_TX[i] = data[i];
	}
	HAL_GPIO_WritePin(MEMS_NSS_GPIO_Port, MEMS_NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive_DMA(&hspi2, MEMS_TX , MEMS_RX, size);
}

void MemsInit(void){
	MemsReadByte( LSM6DS3_WHO_AM_I );
}




void MemsTXRXCallback()
{
	HAL_GPIO_WritePin(Vonal_NSS_GPIO_Port, Vonal_NSS_Pin, GPIO_PIN_SET);
	FlushMemsRx();
	FlushMemsTx();
}
