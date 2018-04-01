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
#include "ssd1306.h"
#include "register_map.h"
#include "menu.h"

#define MAX_RX_BUF 500
#define MSG_LENGTH 166

uint8_t Rx_buf [MAX_RX_BUF];

extern UART_HandleTypeDef huart1;

void uart_startReceiving() {
	__HAL_UART_FLUSH_DRREGISTER(&huart1);
	if( HAL_UART_Receive_IT(&huart1 , Rx_buf , MSG_LENGTH) != HAL_OK)
	{
		ssd1306_WriteErrorMsg("Uart recv error");
	}
}

void uart_send( uint8_t* data , uint8_t size ) {
	int retval = HAL_UART_Transmit(&huart1 , (uint8_t*)data , size , 200);
	if (retval != HAL_OK)
	{
		ssd1306_WriteErrorMsg("Uart send error");
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	process_uart_msg();
	if( HAL_UART_Receive_IT(&huart1 , Rx_buf , MSG_LENGTH) != HAL_OK)
	{
		ssd1306_WriteErrorMsg("Uart recv error");
	}
	RefreshScreen();
}

inline void process_uart_msg()		//beérkezõ parancsok feldolgozása
{
	memcpy( &DebugMode, &Rx_buf[4] , sizeof(DebugMode));
	memcpy( &Running, &Rx_buf[5] , sizeof(Running));
	memcpy( &Mode, &Rx_buf[6] , sizeof(Mode));
	memcpy( &GyorsasagiState, &Rx_buf[7] , sizeof(GyorsasagiState));
	memcpy( &UgyessegiState, &Rx_buf[8] , sizeof(UgyessegiState));
	memcpy( &Timestamp, &Rx_buf[9] , sizeof(Timestamp));
	memcpy( &TimeBetweenData, &Rx_buf[17] , sizeof(TimeBetweenData));
	memcpy( &LinePos_controller, &Rx_buf[21] , sizeof(LinePos_controller));
	memcpy( &LinePositionBack, &Rx_buf[25] , sizeof(LinePositionBack));
	memcpy( &Vonalszog_controller, &Rx_buf[29] , sizeof(Vonalszog_controller));
	memcpy( &LineNumFront, &Rx_buf[33] , sizeof(LineNumFront));
	memcpy( &LineNumBack, &Rx_buf[34] , sizeof(LineNumBack));
	memcpy( &LinearX, &Rx_buf[35] , sizeof(LinearX));
	memcpy( &LinearY, &Rx_buf[39] , sizeof(LinearY));
	memcpy( &LinearZ, &Rx_buf[43] , sizeof(LinearZ));
	memcpy( &AngularX, &Rx_buf[47] , sizeof(AngularX));
	memcpy( &AngularY, &Rx_buf[51] , sizeof(AngularY));
	memcpy( &AngularZ, &Rx_buf[55] , sizeof(AngularZ));
	memcpy( &DistanceFront, &Rx_buf[59] , sizeof(DistanceFront));
	memcpy( &DistanceLeft, &Rx_buf[63] , sizeof(DistanceLeft));
	memcpy( &DistanceRight, &Rx_buf[67] , sizeof(DistanceRight));
	memcpy( &Speed, &Rx_buf[71] , sizeof(Speed));
	memcpy( &Uthossz, &Rx_buf[75] , sizeof(Uthossz));
	memcpy( &MotorCurrent, &Rx_buf[79] , sizeof(MotorCurrent));
	memcpy( &SpeedSP, &Rx_buf[83] , sizeof(SpeedSP));
	memcpy( &TclMotor, &Rx_buf[87] , sizeof(TclMotor));
	memcpy( &SpeedSPGyors, &Rx_buf[89] , sizeof(SpeedSPGyors));
	memcpy( &SpeedSPKanyar, &Rx_buf[93] , sizeof(SpeedSPKanyar));
	memcpy( &KpGyors, &Rx_buf[97] , sizeof(KpGyors));
	memcpy( &KpKanyar, &Rx_buf[101] , sizeof(KpKanyar));
	memcpy( &KdGyors, &Rx_buf[105] , sizeof(KdGyors));
	memcpy( &BatteryMotor, &Rx_buf[109] , sizeof(BatteryMotor));
	memcpy( &BatteryLogic, &Rx_buf[113] , sizeof(BatteryLogic));
	memcpy( &ServoPos, &Rx_buf[117] , sizeof(ServoPos));
	memcpy( &Kp, &Rx_buf[121] , sizeof(Kp));
	memcpy( &Ki, &Rx_buf[125] , sizeof(Ki));
	memcpy( &Kd, &Rx_buf[129] , sizeof(Kd));
	memcpy( &kszi, &Rx_buf[133] , sizeof(kszi));
	memcpy( &OutputDivisor, &Rx_buf[137] , sizeof(OutputDivisor));
	memcpy( &D5percent, &Rx_buf[141] , sizeof(D5percent));
	memcpy( &D5Add, &Rx_buf[145] , sizeof(D5Add));
	memcpy( &D5Mul, &Rx_buf[149] , sizeof(D5Mul));
	memcpy( &KpWeight, &Rx_buf[153] , sizeof(KpWeight));
	memcpy( &KdeltaWeight, &Rx_buf[157] , sizeof(KdeltaWeight));
	memcpy( &KorforgalomData, &Rx_buf[161] , sizeof(KorforgalomData));

	if(Mode == GYORSASAGI)
		strncpy(str_mode , "gyors" , sizeof(str_mode));
	else if( Mode == UGYESSEGI)
		strncpy(str_mode , "ugyes" , sizeof(str_mode));

	if(Running == RUN_FULL_AUTO)
		strncpy(str_control , "auto" , sizeof(str_control));
	else if(Running == RUN_MANUAL_STEERING)
		strncpy(str_control , "man str" , sizeof(str_control));
	else if(Running == RUN_MANUAL_THROTTLE)
		strncpy(str_control , "man thr" , sizeof(str_control));
	else if(Running == RUN_FULL_MANUAL)
		strncpy(str_control , "manual" , sizeof(str_control));
	else if(Running == RUN_STOP)
			strncpy(str_control , "stop" , sizeof(str_control));


}

