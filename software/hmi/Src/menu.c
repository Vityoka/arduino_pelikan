#include "menu.h"
#include "ssd1306.h"

const char *menus[] = {
"StartStop",
"Debug",
"Calibrate",
"Battery",
"Distance",
"Accelerat",
"Status"
};

uint8_t MenuCurrent=0;
uint8_t MenuSub=0;
uint8_t MenuSubMax=0;


uint8_t started=0;
uint8_t BatteryUnit=0;

void MenuNext()
{
	MenuCurrent++;
	if(MenuCurrent >= (sizeof(menus)/sizeof(menus[0])))
		MenuCurrent = 0;
	MenuDraw(MenuCurrent);
}

void MenuPrev()
{
	MenuCurrent--;
	if(MenuCurrent >= (sizeof(menus)/sizeof(menus[0])))
		MenuCurrent = (sizeof(menus)/sizeof(menus[0])) - 1;
	MenuDraw(MenuCurrent);
}

void MenuUp()
{
	if(MenuSub<MenuSubMax)
		MenuSub++;
}

void MenuDown()
{
	if(MenuSub)
		MenuSub--;
}

void MenuDraw(uint8_t MenuCurrent)
{
	//Erase Previous Screen
	ssd1306_Fill(Black);

	//Draw headline
	ssd1306_WriteHeadline(menus[MenuCurrent]);

	//Draw submenus
	switch(MenuCurrent)
	{
	case 0:	//"StartStop"
		ssd1306_SetCursor(16,16);
		ssd1306_WriteString("Start",Font_11x18,White);
		ssd1306_SetCursor(16,32);
		ssd1306_WriteString("Stop",Font_11x18,White);
		ssd1306_SetCursor(0,47);
		if(started)
			ssd1306_WriteString("Now: Running",Font_11x16,White);
		else
			ssd1306_WriteString("Now: Stopped",Font_11x16,White);
		break;

	case 1:	//"Debug"
		ssd1306_SetCursor(16,16);
		ssd1306_WriteString("Turn ON",Font_11x18,White);
		ssd1306_SetCursor(16,32);
		ssd1306_WriteString("Turn OFF",Font_11x18,White);
		ssd1306_SetCursor(0,48);
		if(started)
			ssd1306_WriteString("Debug ON",Font_11x16,White);
		else
			ssd1306_WriteString("Debug OFF",Font_11x16,White);
		break;

	case 2:	//"Calibrate"
		ssd1306_SetCursor(16,16);
		ssd1306_WriteString("Set Dark",Font_11x18,White);
		ssd1306_SetCursor(16,32);
		ssd1306_WriteString("Set Light",Font_11x18,White);
		ssd1306_SetCursor(0,51);
		ssd1306_WriteString("Calibration:",Font_7x10,White);
		ssd1306_SetCursor(88,48);
		if(started)
			ssd1306_WriteString("OK",Font_11x16,White);
		else
			ssd1306_WriteString("ERR",Font_11x16,White);
		break;

	case 3:	// "Battery"
		if(BatteryUnit)
		{
			ssd1306_SetCursor(16,16);
			ssd1306_WriteString("M: TODO",Font_11x18,White);
			ssd1306_SetCursor(16,32);
			ssd1306_WriteString("L: TODO",Font_11x18,White);
		}
		else
		{
			ssd1306_SetCursor(16,16);
			ssd1306_WriteString("M: TODO",Font_11x18,White);
			ssd1306_SetCursor(16,32);
			ssd1306_WriteString("L: TODO",Font_11x18,White);
		}
		ssd1306_SetCursor(0,48);
		ssd1306_WriteString("Set % or V",Font_11x16,White);
		break;

	case 4:	//"Distance"
		ssd1306_SetCursor(16,16);
		ssd1306_WriteString("F: TODO",Font_11x18,White);
		ssd1306_SetCursor(16,32);
		ssd1306_WriteString("L: TODO",Font_11x18,White);
		ssd1306_SetCursor(16,48);
		ssd1306_WriteString("R: TODO",Font_11x16,White);
		break;

	case 5:	//"Accelerat"
		ssd1306_SetCursor(16,16);
		ssd1306_WriteString("X: TODO",Font_11x18,White);
		ssd1306_SetCursor(16,32);
		ssd1306_WriteString("Y: TODO",Font_11x18,White);
		ssd1306_SetCursor(16,48);
		ssd1306_WriteString("Z: TODO",Font_11x16,White);
		break;

	case 6:	//"Status"
		ssd1306_SetCursor(16,16);
		ssd1306_WriteString("TODO",Font_11x18,White);
		ssd1306_SetCursor(16,32);
		ssd1306_WriteString("TODO",Font_11x18,White);
		ssd1306_SetCursor(16,48);
		ssd1306_WriteString("TODO",Font_11x16,White);
		break;
	}
	ssd1306_UpdateScreen();
}
