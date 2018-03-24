#include "MAX7219.h"

static uint8_t Displays[NUMBER_OF_DISPLAYS][8];

//Initialize variables and registers
void InitMAX7219()
{
	// initialize registers
	setRegister(REG_SCANLIMIT, 0x07);	// use all rows/digits
	setRegister(REG_INTENSITY, 0x01);	// brightness (0x00 -> 0x0F)
	setRegister(REG_SHUTDOWN, 0x01);	// normal operation
	setRegister(REG_DECODEMODE, 0x00);	// pixels not integers
	setRegister(REG_DISPLAYTEST, 0x00);	// not in test mode

	//Clear all
	clear();
}

//test function to show off buttons
void testbutton(uint8_t yx)
{
	for(uint8_t i = 0; i<NUMBER_OF_DISPLAYS; i++)
	{
		for(uint8_t row = 0; row<8; row++)
		{
			if(row == ( (yx >> 4) & 0x07 ))
				Displays[i][row]= 1<<((yx>>1) & 0x07);
			else
				Displays[i][row]=0x00;
		}
	}
	updateDisplays();
}

//Update given row of all displays
void updateRow(uint8_t row)
{
	for(uint8_t i = 0; i<NUMBER_OF_DISPLAYS; i++)
	{
		uint8_t address = row+1;
		uint8_t data = Displays[i][row];
		setRegister(address, data);
	}
}

//Update all displays
void updateDisplays()
{
	for(uint8_t row = 0; row<8; row++)
	{
		updateRow(row);
	}
}

//Send data to device
void setRegister(uint8_t address, uint8_t data) {
	//Data format:

	//	||=====================================================================================||
	//	|| D15 | D14 | D13 | D12 | D11 | D10 | D9 | D8 | D7 | D6 | D5 | D4 | D3 | D2 | D1 | D0 ||
	//	||  X  |  X  |  X  |  X  |       ADDRESS       |MSB              DATA               LSB||
	//	||=====================================================================================||

	HAL_GPIO_WritePin(SZEM_NSS_GPIO_Port,SZEM_NSS_Pin,0);
	HAL_SPI_Transmit(&MAX7219_SPI_PORT, &address, 1, 1000);
	HAL_SPI_Transmit(&MAX7219_SPI_PORT, &data, 1, 1000);
	HAL_GPIO_WritePin(SZEM_NSS_GPIO_Port,SZEM_NSS_Pin,1);
}

//Clear buffers and registers
void clear()
{
	//Clear display buffer
	for(uint8_t i = 0; i<NUMBER_OF_DISPLAYS; i++)
	{
		for(uint8_t row = 0; row<8; row++)
		{
			Displays[i][row]=0x00;
		}
	}

	//Clear registers
	updateDisplays();
}

