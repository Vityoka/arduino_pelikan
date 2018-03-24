#include "ssd1306.h"
#include <string.h>

static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

static SSD1306_t SSD1306;

//I2C write command
void ssd1306_WriteCommand(uint8_t command)
{
	HAL_I2C_Mem_Write(&SSD1306_I2C_PORT, (SSD1306_I2C_ADDR << 1), 0x00, 1, &command, 1, 100);
}
//Initializing SSD1306 chip
uint8_t ssd1306_Init(void)
{
	HAL_Delay(100);
	
	ssd1306_WriteCommand(0xAE);	//Display OFF (sleep mode)

	ssd1306_WriteCommand(0xD5);	// Osclillator freq and CLK divide ratio
	ssd1306_WriteCommand(0x80);	//the suggested ratio 0x80
	
	ssd1306_WriteCommand(0xA8);	//Mux Ratio (16MUX -> 64MUX)
	ssd1306_WriteCommand(0x3F);	//64MUX
	
	ssd1306_WriteCommand(0xD3);	//Vertical offset
	ssd1306_WriteCommand(0x0);	//no offset
	
	ssd1306_WriteCommand(0x40);	//Set Start line register to 0
	
	ssd1306_WriteCommand(0x20);	//Memory Addressing Mode
	ssd1306_WriteCommand(0x00);	//Horizontal Addressing Mode
	
	ssd1306_WriteCommand(0xA1);	//Column address 127 is mapped to SEG0
	
	ssd1306_WriteCommand(0xC8);	//COM Output: remapped scan dir. (0xC0->normal) (vertical flip?)

	ssd1306_WriteCommand(0xDA);	//Set COM Pins Hardware Configuration
	ssd1306_WriteCommand(0x12); //0b00??0010
	
	ssd1306_WriteCommand(0x81);	//Set Contrast
	ssd1306_WriteCommand(0x9F);	//random contrast value

	ssd1306_WriteCommand(0xD9);	//Set Pre-charge Period
	ssd1306_WriteCommand(0xF1);	//
	
	ssd1306_WriteCommand(0xDB);	//Set VCOMH Deselect Level
	ssd1306_WriteCommand(0x40);	//?
	
	ssd1306_WriteCommand(0xA4);	//Normal / Entire Display STAY ON

	ssd1306_WriteCommand(0xA6);	//A6->Normal A7->Inverse Display

	ssd1306_WriteCommand(0x2E);	//Deactivate scrolling (2F->Activate)

	ssd1306_WriteCommand(0x8D);	//Charge Pump settings
	ssd1306_WriteCommand(0x14);	//Enable Charge Pump
	ssd1306_WriteCommand(0xAF);	//Display ON

/* Initialized OK */
SSD1306.Initialized = 1;

/* Return OK */
return 1;
}

//Fill the screen
void ssd1306_Fill(SSD1306_COLOR color)
{
	/* Set memory */
	uint32_t i;

	for(i = 0; i < sizeof(SSD1306_Buffer); i++)
	{
		SSD1306_Buffer[i] = (color == Black) ? 0x00 : 0xFF;
	}
}

//Update RAM to screen
void ssd1306_UpdateScreen(void)
{
	uint8_t i;

	for (i = 0; i < 8; i++) {
		ssd1306_WriteCommand(0xB0 + i);
		ssd1306_WriteCommand(0x00);
		ssd1306_WriteCommand(0x10);

		HAL_I2C_Mem_Write(&SSD1306_I2C_PORT,(SSD1306_I2C_ADDR << 1), 0x40, 1, &SSD1306_Buffer[SSD1306_WIDTH * i], SSD1306_WIDTH, 100 );
	}
}


//Draw one pixel
void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color)
{
	if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT)
	{
		return;
	}

	if (color == White)
	{
		SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
	}
	else
	{
		SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
	}
}


char ssd1306_WriteChar(char ch, FontDef Font, SSD1306_COLOR color)
{
	uint32_t i, b, j;


	if (SSD1306_WIDTH < (SSD1306.CurrentX + Font.FontWidth) ||
		SSD1306_HEIGHT < (SSD1306.CurrentY + Font.FontHeight))
	{

		return 0;
	}


	for (i = 0; i < Font.FontHeight; i++)
	{
		b = Font.data[(ch - 32) * Font.FontHeight + i];
		for (j = 0; j < Font.FontWidth; j++)
		{
			if ((b << j) & 0x8000)
			{
				ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR) color);
			}
			else
			{
				ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR)!color);
			}
		}
	}


	SSD1306.CurrentX += Font.FontWidth;

	return ch;
}


// Writing string function
char ssd1306_WriteString(char* str, FontDef Font, SSD1306_COLOR color)
{
	while (*str)
	{
		ssd1306_WriteChar(*str, Font, color);
		str++;
	}

	return *str;
}


//Set cursor coordinate
void ssd1306_SetCursor(uint8_t x, uint8_t y)
{
	/* Set write pointers */
	SSD1306.CurrentX = x;
	SSD1306.CurrentY = y;
}

//Write headline in menu
void ssd1306_WriteHeadline(char* str)
{
	FontDef DefaultFont = Font_11x16;
	SSD1306_COLOR DefaultColor = White;

	//Erase Headline Area
	/*
	uint32_t i;
	for(i = 0; i < SSD1306_WIDTH * 16 / 8 ; i++)
	{
		SSD1306_Buffer[i] = (DefaultColor == White) ? 0x00 : 0xFF;
	}
	*/

	//Top of screen
	SSD1306.CurrentY = 0;

	//Print Arrows
	SSD1306.CurrentX = 0;
	ssd1306_WriteChar('<', DefaultFont, DefaultColor);

	SSD1306.CurrentX = SSD1306_WIDTH-12;
	ssd1306_WriteChar('>',DefaultFont, DefaultColor);

	//Center text
	uint8_t TextLengthPx = strlen(str) * DefaultFont.FontWidth;

		//Text too long -> truncate
		if(TextLengthPx > 128)
		{
			TextLengthPx = 99;
		}

	SSD1306.CurrentX = (SSD1306_WIDTH - TextLengthPx) / 2;

	//Print Text
	uint8_t j=0;
	while (str[j] && (j<9))
	{
		ssd1306_WriteChar(str[j], DefaultFont, DefaultColor);
		j++;
	}
}
