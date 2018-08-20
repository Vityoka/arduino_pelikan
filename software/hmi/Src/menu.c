#include "menu.h"
#include "ssd1306.h"
#include "register_map.h"
#include "buttons.h"
#include "string.h"
#include "uartmain.h"

const char *menus[] = {
"Modes",
"Ctrl",
"LineSens",
"Speed",
"Distance",
"IMU",
"RPi",
"DirCntrl"
};

uint8_t MenuCurrent=0;
uint8_t MenuSub=0;
uint8_t MenuSubMax=0;

uint8_t started=0;
uint8_t BatteryUnit=0;

int mystrlen(char* str){	//addig számol amig nemjön lezáró \n
	int i = 0;
	for(i = 0; str[i] != '\n'; i++);
	return i+1;
}


void RefreshScreen()
{
	MenuDraw(MenuCurrent);
}

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
	char buf [50];

	//Erase Previous Screen
	ssd1306_Fill(Black);

	//Draw headline
	ssd1306_WriteHeadline(menus[MenuCurrent]);

	//Draw submenus
	switch(MenuCurrent)
	{
	case 0:	//"Modes"
		sprintf(buf, "Mode: %s" , str_mode );
		ssd1306_SetCursor(16,16);
		ssd1306_WriteString(buf,Font_7x10,White);
		sprintf(buf, "Ctrl: %s" , str_control );
		ssd1306_SetCursor(16,32);
		ssd1306_WriteString(buf,Font_7x10,White);
		sprintf(buf, "STOP" );
		ssd1306_SetCursor(16,48);
		ssd1306_WriteString(buf,Font_7x10,White);
		break;

	case 1:	//"Ctrl"

		sprintf(buf, "%2.2f,%2.2f" , OutputDivisor, D5percent );
		ssd1306_SetCursor(16,16);
		ssd1306_WriteString(buf,Font_7x10,White);
		sprintf(buf, "%2.2f,%2.2f" , D5Add,D5Mul );
		ssd1306_SetCursor(16,28);
		ssd1306_WriteString(buf,Font_7x10,White);
		sprintf(buf, "%2.2f,%2.2f" , KpWeight,KdeltaWeight);
		ssd1306_SetCursor(16,38);
		ssd1306_WriteString(buf,Font_7x10,White);
		sprintf(buf, "%2.2f,%2.2f" , Kd ,kszi);
		ssd1306_SetCursor(16,50);
		ssd1306_WriteString(buf,Font_7x10,White);
		break;

	case 2:	//"LineSens"
		sprintf(buf, "LinePos  LineNum" );
		ssd1306_SetCursor(16,16);
		ssd1306_WriteString(buf,Font_7x10,White);
		sprintf(buf, "F: %2.1f , %d" , LinePos_controller, LineNumFront );
		ssd1306_SetCursor(16,28);
		ssd1306_WriteString(buf,Font_7x10,White);
		sprintf(buf, "B: %2.1f , %d" , LinePositionBack, LineNumBack );
		ssd1306_SetCursor(16,38);
		ssd1306_WriteString(buf,Font_7x10,White);
		sprintf(buf, "Angle: %2.1f deg" , Vonalszog_controller*180/3.1415 );
		ssd1306_SetCursor(16,50);
		ssd1306_WriteString(buf,Font_7x10,White);
		break;

	case 3:	// "Speed"
		sprintf(buf, "Speed: %2.1f" , Speed );
		ssd1306_SetCursor(16,16);
		ssd1306_WriteString(buf,Font_7x10,White);
		sprintf(buf, "Uthossz: %2.1f" , Uthossz );
		ssd1306_SetCursor(16,32);
		ssd1306_WriteString(buf,Font_7x10,White);
		sprintf(buf, "SpeedSP: %2.1f" , SpeedSP );
		ssd1306_SetCursor(16,48);
		ssd1306_WriteString(buf,Font_7x10,White);
		break;

	case 4:	//"Distance"
		sprintf(buf, "F: %2.1f" , DistanceFront );
		ssd1306_SetCursor(16,16);
		ssd1306_WriteString(buf,Font_11x18,White);
		sprintf(buf, "L: %2.1f" , DistanceLeft );
		ssd1306_SetCursor(16,32);
		ssd1306_WriteString(buf,Font_11x18,White);
		sprintf(buf, "R: %2.1f" , DistanceRight );
		ssd1306_SetCursor(16,48);
		ssd1306_WriteString(buf,Font_11x16,White);
		break;

	case 5:	//"IMU"
		sprintf(buf, "Accel    Gyro" );
		ssd1306_SetCursor(16,16);
		ssd1306_WriteString(buf,Font_7x10,White);
		sprintf(buf, "X: %2.1f , %2.1f" , LinearX, AngularX );
		ssd1306_SetCursor(16,28);
		ssd1306_WriteString(buf,Font_7x10,White);
		sprintf(buf, "Y: %2.1f , %2.1f" , LinearY, AngularY );
		ssd1306_SetCursor(16,38);
		ssd1306_WriteString(buf,Font_7x10,White);
		sprintf(buf, "Z: %2.1f , %2.1f" , LinearZ, AngularZ );
		ssd1306_SetCursor(16,50);
		ssd1306_WriteString(buf,Font_7x10,White);
		break;

	case 6:	//"Rpi"
		sprintf(buf, "Rpi0: %d ", rpi_rx_0 );
		ssd1306_SetCursor(16,16);
		ssd1306_WriteString(buf ,Font_11x18,White);
		sprintf(buf, "Rpi1: %d ", rpi_rx_1 );
		ssd1306_SetCursor(16,32);
		ssd1306_WriteString(buf ,Font_11x18,White);
		ssd1306_SetCursor(16,48);
		ssd1306_WriteString("TODO",Font_11x16,White);
		break;

	case 7:	//"Direct Control"
		sprintf(buf, "SpeedSP 0.1" );
		ssd1306_SetCursor(16,16);
		ssd1306_WriteString(buf,Font_7x10,White);
		sprintf(buf, "SpeedSP 0.5" );
		ssd1306_SetCursor(16,32);
		ssd1306_WriteString(buf,Font_7x10,White);
		sprintf(buf, "STOP" );
		ssd1306_SetCursor(16,48);
		ssd1306_WriteString(buf,Font_7x10,White);
		break;
	}
	ssd1306_UpdateScreen();
}

void MenuLogic(uint8_t Menupont)
{
	// Command formátuma a MainBoard felé:
	// Elsõ 4 bájt az üzenet hossza bájtban.
	// Ezután egy darab karakter ami a commandot határozza meg. Mainboard oldalán van definiálva
	// Ezután az adott commandhoz szükséges extra infok küldése
	uint8_t cmd [10];
	uint8_t temp;
	float floattemp;
	switch(MenuCurrent)
	{
	case 0:		//"StartStop"
		if(Menupont == 1)
		{
			temp = Mode;
			if (temp == 1)
			{
				uint8_t tempcmd [] =  "\0\0\0\5Y\n";
				memcpy(cmd , tempcmd , sizeof(tempcmd));
				temp = 0;
			}
			else if(temp == 0)
			{
				uint8_t tempcmd [] =  "\0\0\0\5U\n";
				memcpy(cmd , tempcmd , sizeof(tempcmd));
				temp = 1;
			}
		}
		else if(Menupont == 2)
		{
			temp = Running;
			if( Running >= 5)
				temp = 1;
			else
				temp++;
			uint8_t tempcmd [] =  "\0\0\0\6G\0\n";
			memcpy(cmd , tempcmd, sizeof(tempcmd));
			cmd[5] = temp;
		}
		else if(Menupont == 3)
		{
			uint8_t tempcmd [] =  "\0\0\0\5S\n";
			memcpy(cmd , tempcmd , sizeof(tempcmd));
		}
		uart_send(cmd , mystrlen(cmd) );
		memset(cmd, 0, sizeof(cmd));
		break;
	case 1:
		break;
	case 2:
		break;
	case 3:
		break;
	case 4:
		break;
	case 5:
		break;
	case 6:
		break;
	case 7:	//Direct Control , setting SpeedSP

		if(Menupont == 1)
		{
			floattemp = 0.1;
			uint8_t tempcmd [] =  "\0\0\0______\n";
			tempcmd[3] = 9;
			tempcmd[4] = 'V';
			memcpy(cmd , tempcmd , sizeof(tempcmd));
			memcpy(&cmd[5] , &floattemp , sizeof(floattemp));
		}
		if(Menupont == 2)
		{
			floattemp = 0.5;
			uint8_t tempcmd [] =  "\0\0\0______\n";
			tempcmd[3] = 9;
			tempcmd[4] = 'V';
			memcpy(cmd , tempcmd , sizeof(tempcmd));
			memcpy(&cmd[5] , &floattemp , sizeof(floattemp));
		}
		if(Menupont == 3)
		{
			uint8_t tempcmd [] =  "\0\0\0\5S\n";
			memcpy(cmd , tempcmd , sizeof(tempcmd));
		}

		uart_send(cmd , mystrlen(cmd) );
		memset(cmd, 0, sizeof(cmd));
		break;

	}

}





