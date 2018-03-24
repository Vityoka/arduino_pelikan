/*
 * SPIinfra.cpp
 *
 *  Created on: 2017. nov. 4.
 *      Author: Vity�
 */

#include <SPI_infra.h>
#include "main.h"
#include "stm32f0xx_hal.h"

extern SPI_HandleTypeDef hspi1;
extern volatile uint8_t infra_leds_ready_flag;

SPI_infra::SPI_infra() {
	HAL_GPIO_WritePin(infra_OE_GPIO_Port , infra_OE_Pin , GPIO_PIN_SET);	//kimenet enged�lyez�se, 0-ra akt�v
	HAL_GPIO_WritePin(infra_LE_GPIO_Port , infra_LE_Pin , GPIO_PIN_RESET);	//regiszterbe val� �tt�lt�s jel, 1-re akt�v
}

SPI_infra::~SPI_infra() {
	// TODO Auto-generated destructor stub
}

void SPI_infra::send( uint8_t cycle) {

	HAL_GPIO_WritePin(infra_LE_GPIO_Port , infra_LE_Pin , GPIO_PIN_RESET);	//alaphelyzetbe �ll�tom a regiszterbe �tt�lt� jelet
	HAL_GPIO_WritePin(infra_OE_GPIO_Port , infra_OE_Pin , GPIO_PIN_SET);	//tiltom a kimeneteket a shiftel�s idej�re

	uint8_t byte_buffer;
	if( (cycle <= 7) && (cycle >= 0) )
	{
		switch ( cycle )
		{
			case 0:
				byte_buffer = 0b00000001;
				break;
			case 1:
				byte_buffer = 0b00000010;
				break;
			case 2:
				byte_buffer = 0b00000100;
				break;
			case 3:
				byte_buffer = 0b00001000;
				break;
			case 4:
				byte_buffer = 0b00010000;
				break;
			case 5:
				byte_buffer = 0b00100000;
				break;
			case 6:
				byte_buffer = 0b01000000;
				break;
			case 7:
				byte_buffer = 0b10000000;
				break;
		}
	}
	for( int i = 0 ; i < 8 ; i++)
	{
		Tx_buffer[i] = byte_buffer;
	}

	HAL_SPI_Transmit(&hspi1 , (uint8_t*)Tx_buffer , sizeof(Tx_buffer) , 100 );	//blokkol� SPI adatk�ld�s :(

	HAL_GPIO_WritePin(infra_LE_GPIO_Port , infra_LE_Pin , GPIO_PIN_SET);		//ha mind a 64-et kishiftelt�k , �tt�lt�nk a regiszterekbe, 1-re akt�v jel
	HAL_GPIO_WritePin(infra_OE_GPIO_Port , infra_OE_Pin , GPIO_PIN_RESET);		//�jra enged�lyezem a kimeneteket
}

void SPI_infra::debug_infra() {

	uint8_t data = 0x00;
	for(int i = 0 ;  i<8 ; i++)
	{
		HAL_GPIO_WritePin(infra_LE_GPIO_Port , infra_LE_Pin , GPIO_PIN_RESET);	//alaphelyzetbe �ll�tom a regiszterbe �tt�lt� jelet
		HAL_GPIO_WritePin(infra_OE_GPIO_Port , infra_OE_Pin , GPIO_PIN_SET);	//tiltom a kimeneteket a shiftel�s idej�re
		HAL_SPI_Transmit(&hspi1 , &data , 1 , 100 );
		HAL_GPIO_WritePin(infra_LE_GPIO_Port , infra_LE_Pin , GPIO_PIN_SET);		//ha mind a 64-et kishiftelt�k , �tt�lt�nk a regiszterekbe, 1-re akt�v jel
		HAL_GPIO_WritePin(infra_OE_GPIO_Port , infra_OE_Pin , GPIO_PIN_RESET);		//�jra enged�lyezem a kimeneteket
	}

	HAL_GPIO_WritePin(infra_LE_GPIO_Port , infra_LE_Pin , GPIO_PIN_RESET);	//alaphelyzetbe �ll�tom a regiszterbe �tt�lt� jelet
	HAL_GPIO_WritePin(infra_OE_GPIO_Port , infra_OE_Pin , GPIO_PIN_SET);	//tiltom a kimeneteket a shiftel�s idej�re
	data = 0xff;
	HAL_SPI_Transmit(&hspi1 , &data , 1 , 100 );
	HAL_GPIO_WritePin(infra_LE_GPIO_Port , infra_LE_Pin , GPIO_PIN_SET);		//ha mind a 64-et kishiftelt�k , �tt�lt�nk a regiszterekbe, 1-re akt�v jel
	HAL_GPIO_WritePin(infra_OE_GPIO_Port , infra_OE_Pin , GPIO_PIN_RESET);		//�jra enged�lyezem a kimeneteket

	data = 0;
	for( int i = 0 ;  i < 8 ; i++)
	{
		HAL_GPIO_WritePin(infra_LE_GPIO_Port , infra_LE_Pin , GPIO_PIN_RESET);	//alaphelyzetbe �ll�tom a regiszterbe �tt�lt� jelet
		HAL_GPIO_WritePin(infra_OE_GPIO_Port , infra_OE_Pin , GPIO_PIN_SET);	//tiltom a kimeneteket a shiftel�s idej�re
		HAL_SPI_Transmit(&hspi1 , &data , 1 , 100 );
		HAL_GPIO_WritePin(infra_LE_GPIO_Port , infra_LE_Pin , GPIO_PIN_SET);		//ha mind a 64-et kishiftelt�k , �tt�lt�nk a regiszterekbe, 1-re akt�v jel
		HAL_GPIO_WritePin(infra_OE_GPIO_Port , infra_OE_Pin , GPIO_PIN_RESET);		//�jra enged�lyezem a kimeneteket
	}
}

void SPI_infra::debug_infra3() {

	uint8_t data = 0x00;
	for(int i = 0 ;  i<8 ; i++)
	{
		HAL_GPIO_WritePin(infra_LE_GPIO_Port , infra_LE_Pin , GPIO_PIN_RESET);	//alaphelyzetbe �ll�tom a regiszterbe �tt�lt� jelet
		HAL_GPIO_WritePin(infra_OE_GPIO_Port , infra_OE_Pin , GPIO_PIN_SET);	//tiltom a kimeneteket a shiftel�s idej�re
		HAL_SPI_Transmit(&hspi1 , &data , 1 , 100 );
		HAL_GPIO_WritePin(infra_LE_GPIO_Port , infra_LE_Pin , GPIO_PIN_SET);		//ha mind a 64-et kishiftelt�k , �tt�lt�nk a regiszterekbe, 1-re akt�v jel
		HAL_GPIO_WritePin(infra_OE_GPIO_Port , infra_OE_Pin , GPIO_PIN_RESET);		//�jra enged�lyezem a kimeneteket
	}

	data = 0xAA;
	while(1)
	{
		HAL_GPIO_WritePin(infra_LE_GPIO_Port , infra_LE_Pin , GPIO_PIN_RESET);	//alaphelyzetbe �ll�tom a regiszterbe �tt�lt� jelet
		HAL_GPIO_WritePin(infra_OE_GPIO_Port , infra_OE_Pin , GPIO_PIN_SET);	//tiltom a kimeneteket a shiftel�s idej�re
		HAL_SPI_Transmit(&hspi1 , &data , 1 , 100 );
		HAL_GPIO_WritePin(infra_LE_GPIO_Port , infra_LE_Pin , GPIO_PIN_SET);		//ha mind a 64-et kishiftelt�k , �tt�lt�nk a regiszterekbe, 1-re akt�v jel
		HAL_GPIO_WritePin(infra_OE_GPIO_Port , infra_OE_Pin , GPIO_PIN_RESET);		//�jra enged�lyezem a kimeneteket
	}
}

void SPI_infra::debug_infra4() {		//a val�di kommunik�ci�hoz tartoz� k�d, egy v�gtelen ciklusban
	uint8_t transferbuffer[8];
	int cycle = 0;

	while(1)
	{
		uint8_t byte_buffer;
		if( (cycle <= 7) && (cycle >= 0) )
		{
			switch ( cycle )
			{
				case 0:
					byte_buffer = 0x01;
					break;
				case 1:
					byte_buffer = 0x02;
					break;
				case 2:
					byte_buffer = 0x04;
					break;
				case 3:
					byte_buffer = 0x08;
					break;
				case 4:
					byte_buffer = 0x10;
					break;
				case 5:
					byte_buffer = 0x20;
					break;
				case 6:
					byte_buffer = 0x40;
					break;
				case 7:
					byte_buffer = 0x80;
					break;
			}
		}
		for( int i = 0 ; i < 8 ; i++)
		{
			transferbuffer[i] = byte_buffer;
		}

		HAL_GPIO_WritePin(infra_LE_GPIO_Port , infra_LE_Pin , GPIO_PIN_RESET);	//alaphelyzetbe �ll�tom a regiszterbe �tt�lt� jelet
		HAL_GPIO_WritePin(infra_OE_GPIO_Port , infra_OE_Pin , GPIO_PIN_SET);	//tiltom a kimeneteket a shiftel�s idej�re

		HAL_SPI_Transmit(&hspi1 , transferbuffer , 8 , 100 );	//blokkol� SPI adatk�ld�s :(

		HAL_GPIO_WritePin(infra_LE_GPIO_Port , infra_LE_Pin , GPIO_PIN_SET);		//ha mind a 64-et kishiftelt�k , �tt�lt�nk a regiszterekbe, 1-re akt�v jel
		HAL_GPIO_WritePin(infra_OE_GPIO_Port , infra_OE_Pin , GPIO_PIN_RESET);		//�jra enged�lyezem a kimeneteket

		cycle++;
		if( cycle == 8 )
		{
			cycle = 0;
			for( int swdelay = 0; swdelay < 1000 ; swdelay++){}
		}
	}
}

void SPI_infra::debug_infra5() {//v�gtelen ciklusvan, csak nem l�pkedve az �sszes �rt�ken, hanem v�gig egy fix �rt�ket adunk ki. SPI hull�mforma tesztel�shez
	uint8_t transferbuffer[8];
	int cycle = 0;

	while(1)
	{
		uint8_t byte_buffer = 0x80;

		HAL_GPIO_WritePin(infra_LE_GPIO_Port , infra_LE_Pin , GPIO_PIN_RESET);	//alaphelyzetbe �ll�tom a regiszterbe �tt�lt� jelet
		HAL_GPIO_WritePin(infra_OE_GPIO_Port , infra_OE_Pin , GPIO_PIN_SET);	//tiltom a kimeneteket a shiftel�s idej�re

		HAL_SPI_Transmit(&hspi1 , &byte_buffer , 1 , 100 );	//blokkol� SPI adatk�ld�s :(

		HAL_GPIO_WritePin(infra_LE_GPIO_Port , infra_LE_Pin , GPIO_PIN_SET);		//ha mind a 64-et kishiftelt�k , �tt�lt�nk a regiszterekbe, 1-re akt�v jel
		HAL_GPIO_WritePin(infra_OE_GPIO_Port , infra_OE_Pin , GPIO_PIN_RESET);		//�jra enged�lyezem a kimeneteket

		cycle++;
		if( cycle == 8 )
		{
			cycle = 0;
			for( int swdelay = 0; swdelay < 1000 ; swdelay++){}
		}
	}
}

void SPI_infra::debug_infra2() {	//egyes�vel l�peget�nk v�gig az �ssezs infr�n

	uint8_t byte_buffer;
	int loc = 0;
	while(1)
	{
		for(int loc = 7 ; loc >= 0 ; loc--)
		{
			for(int cycle = 0 ;  cycle < 8 ; cycle++ )
			{
				HAL_GPIO_WritePin(infra_LE_GPIO_Port , infra_LE_Pin , GPIO_PIN_RESET);	//alaphelyzetbe �ll�tom a regiszterbe �tt�lt� jelet
				HAL_GPIO_WritePin(infra_OE_GPIO_Port , infra_OE_Pin , GPIO_PIN_SET);	//tiltom a kimeneteket a shiftel�s idej�re

				switch ( cycle )
				{
					case 0:
						byte_buffer = 0b00000001;
						break;
					case 1:
						byte_buffer = 0b00000010;
						break;
					case 2:
						byte_buffer = 0b00000100;
						break;
					case 3:
						byte_buffer = 0b00001000;
						break;
					case 4:
						byte_buffer = 0b00010000;
						break;
					case 5:
						byte_buffer = 0b00100000;
						break;
					case 6:
						byte_buffer = 0b01000000;
						break;
					case 7:
						byte_buffer = 0b10000000;
						break;
				}

				for( int i = 0 ; i < 8 ; i++)
				{
					Tx_buffer[i] = 0;
				}
				Tx_buffer[loc] = byte_buffer;

				HAL_SPI_Transmit(&hspi1 , (uint8_t*)Tx_buffer , sizeof(Tx_buffer) , 100 );

				HAL_GPIO_WritePin(infra_LE_GPIO_Port , infra_LE_Pin , GPIO_PIN_SET);		//ha mind a 64-et kishiftelt�k , �tt�lt�nk a regiszterekbe, 1-re akt�v jel
				HAL_GPIO_WritePin(infra_OE_GPIO_Port , infra_OE_Pin , GPIO_PIN_RESET);		//�jra enged�lyezem a kimeneteket
			}
		}
	}
}
