/*
 * mems.c
 *
 *  Created on: Nov 7, 2017
 *      Author: smate
 */

#include "register_map.h"
#include "rpi.h"

extern SPI_HandleTypeDef hspi2;


uint8_t SPI_RX[SPI_RX_SIZE];
uint8_t SPI_TX[SPI_TX_SIZE];

void RPiSPIRxFlush(){
	for(int i = 0; i < SPI_RX_SIZE ; i++)
	{
		SPI_RX[i] = 0;
	}
}

void RPiSPITxFlush(){
	for(int i = 0; i < SPI_TX_SIZE ; i++)
	{
		SPI_TX[i] = 0;
	}
}

void RpiListen(){
	RPiSPIRxFlush();
	if( HAL_SPI_TransmitReceive_DMA(&hspi2, SPI_TX , SPI_RX, 2) != HAL_OK )
	{
		asm("bkpt 255");
	}
}

void RPiSPICallback()
{
	UBaseType_t uxSavedInterruptStatus;
	uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
	for(int i = 0 ; i <SPI_RX_SIZE ; i++)
	{
		rpi_rx[i] = SPI_RX[i];
	}
	taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
	RPiSPIRxFlush();
	if( HAL_SPI_TransmitReceive_DMA(&hspi2, SPI_TX , SPI_RX, 2) != HAL_OK )
	{
		asm("bkpt 255");
	}
}
