/*
 * bluetooth.c
 *
 *  Created on: Nov 7, 2017
 *      Author: smate
 */


#include "bluetooth.h"
#include "string.h"
#include "controller.h"
#include "main.h"
#include "hmi.h"
#include "starter.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart4;
extern volatile uint8_t hmi_buffer_tx[HMI_BUFF_TX_SIZE];

void BTCommunicationTask(void const * argument)
{
	uint32_t TimeElapsedFromStart = HAL_GetTick();
	while(1)
	{
		if(strncmp(bt_buffer_rx, "AT-AB SPPConnectionClosed", 25) == 0)
		{
			BT_CONNECTED = 0x00;	//connected false
			BluetoothFlushRX();
		}
		else if(strncmp(bt_buffer_rx, "AT-AB ConnectionUp", 18) == 0)
		{
			BT_CONNECTED = 0xFF;	//connected true
			BluetoothFlushRX();
		}
		else if(strncmp(bt_buffer_rx, "AT-AB", 5) == 0)
		{
			BluetoothFlushRX();
		}
		else if(BT_CONNECTED == 0xFF)	//BT serial port connected
		{
			CommandParser(bt_buffer_rx , rx_pointer);
			BluetoothFlushRX();
			Timestamp++;
			TimeBetweenData = HAL_GetTick() - TimeElapsedFromStart;
			TimeElapsedFromStart = HAL_GetTick();
			if(DebugMode)
			{
				//LoadBufferDebug();
			}
			else
			{
				int size = LoadBuffer(bt_buffer_tx);
				BluetoothSend(size);
			}

		}
		else
		{

		}
		osDelay(100);
	}
	osThreadTerminate(NULL);
}

void BluetoothResetCallback()
{
	BluetoothInit(&huart1);
}

void BluetoothInit(UART_HandleTypeDef * UartHandle)
{
	BtUartHandle = UartHandle;
	BluetoothFlushRX();
	BluetoothFlushTX();
	BT_CONNECTED = 0x00;
	bt_rx = 0;
	rx_pointer = 0;
	return;
}

void BluetoothSend(uint16_t size)
{
	HAL_UART_Transmit_DMA(BtUartHandle, bt_buffer_tx, size);
	return;
}

void BluetoothReceive(void)
{
	HAL_UART_Receive_DMA(BtUartHandle, &bt_rx, 1 );
	return;
}

void BluetoothFlushTX(void)
{
	unsigned short i=0;
	for(i=0; i<BT_BUFF_TX_SIZE; i++)
	{
		bt_buffer_tx[i] = 0;
	}
	return;
}

void BluetoothFlushRX(void)
{
	unsigned short i=0;
	rx_pointer = 0;
	for(i=0; i<BT_BUFF_RX_SIZE; i++)
	{
		bt_buffer_rx[i] = 0;
	}
	return;
}

void CommandParser(uint8_t* rxbuf , uint16_t rx_index )
{
	if(rxbuf[4] != 0)
	{
		uint32_t commandSize = 0;
		commandSize |= (rxbuf[0] << 24);
		commandSize |= (rxbuf[1] << 16);
		commandSize |= (rxbuf[2] << 8);
		commandSize |= rxbuf[3];
		if( rx_index < commandSize)
		{
			return;
		}
		else
		{
			switch(rxbuf[4])
			{
				case 'G':	// run
					Running = rxbuf[5];
					break;
				case 'S':	// stop
					Running = RUN_STOP;
					SpeedSP = 0;
					break;
				case 'O':	//debug on
					DebugMode = 0xFF;
					DebugFirstRun = 0xFF;
					break;
				case 'N':	//debug off
					DebugMode = 0x00;
					DebugFirstRun = 0xFF;
					break;
				case 'V':	//speed setpoint
					SpeedSP = 0;
					memcpy(&SpeedSP, (rxbuf + 5), 4);
					break;
				case 'T':	//timestamp
					Timestamp = 0;
					Timestamp |= rxbuf[5];
					Timestamp <<= 8;
					Timestamp |= rxbuf[6];
					Timestamp <<= 8;
					Timestamp |= rxbuf[7];
					Timestamp <<= 8;
					Timestamp |= rxbuf[8];
					Timestamp <<= 8;
					Timestamp |= rxbuf[9];
					Timestamp <<= 8;
					Timestamp |= rxbuf[10];
					Timestamp <<= 8;
					Timestamp |= rxbuf[11];
					Timestamp <<= 8;
					Timestamp |= rxbuf[12];
					break;
				case 'L':	//calibrate light
					CalibRequestState = 1;
					break;
				case 'D':	//calibrate dark
					break;
				case '1':	//Kp
					Kp = 0;
					memcpy(&Kp, (rxbuf + 5), 4);
					break;
				case '2':	//Ki
					Ki = 0;
					memcpy(&Ki, (rxbuf + 5), 4);
					break;
				case '3':	//Kd
					Kd = 0;
					memcpy(&Kd, (rxbuf + 5), 4);
					break;
				case 'q':	//KpGyors
					KpGyors = 0;
					memcpy(&KpGyors, (rxbuf + 5), 4);
					break;
				case 'w':	//KdGyors
					KdGyors = 0;
					memcpy(&KdGyors, (rxbuf + 5), 4);
					break;
				case 'e':	//KpKanyar
					KpKanyar = 0;
					memcpy(&KpKanyar, (rxbuf + 5), 4);
					break;
				case 'r':	//KdKanyar
					KdKanyar = 0;
					memcpy(&KdKanyar, (rxbuf + 5), 4);
					break;
				case 'X':	//selftest
					SELFTEST = 0xFF;
					break;
				case 'U':	//ügyességi mód
					if( Mode != UGYESSEGI )
						Mode = UGYESSEGI;
					//SetTasks();
					break;
				case 'Y':	//gyorsasági mód
					if( Mode != GYORSASAGI)
						Mode = GYORSASAGI;
					//SetTasks();
					break;
				case 'l':	//TclMotor
					TclMotor = 0;
					TclMotor |= (rxbuf[5] << 8);
					TclMotor |= rxbuf[6];
					break;
				case 'u':	//uthossz nullazasa
					Uthossz = 0;
					break;
				case 'y':	//ugyessegi állapotgép
					UgyessegiState = rxbuf[5];
					break;
				case 'g':	//gyorsasági állapotgép
					GyorsasagiState = rxbuf[5];
					break;
				case 'k':	//kszi
					kszi = 0;
					memcpy(&kszi, (rxbuf + 5), 4);
					break;
				case '*':	//D5Mul
					D5Mul = 0;
					memcpy(&D5Mul, (rxbuf + 5), 4);
					break;
				case '+':	//D5Add
					D5Add = 0;
					memcpy(&D5Add, (rxbuf + 5), 4);
					break;
				case 's':	//ServoPos
					ServoPos = 0;
					memcpy(&ServoPos, (rxbuf + 5), 4);
					break;
				case 'o':	//OutputDivisor
					OutputDivisor = 0;
					memcpy(&OutputDivisor, (rxbuf + 5), 4);
					break;
				case 'p':	//KpWeight
					KpWeight = 0;
					memcpy(&KpWeight, (rxbuf + 5), 4);
					break;
				case 'd':	//KdeltaWeight
					KdeltaWeight = 0;
					memcpy(&KdeltaWeight, (rxbuf + 5), 4);
					break;
				case '6':	//SpeedSPGyors
					SpeedSPGyors = 0;
					memcpy(&SpeedSPGyors, (rxbuf + 5), 4);
					break;
				case '7':	//SpeedSPKanyar
					SpeedSPKanyar = 0;
					memcpy(&SpeedSPKanyar, (rxbuf + 5), 4);
					break;

				default:
					return;
			}

		}
	}
	return;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if( huart->Instance == UART4 )
	{
		HMIUartRxCallback();
	}
	else if( huart->Instance == USART1 )
	{
		bt_buffer_rx[rx_pointer ] = bt_rx;
		rx_pointer++;
		bt_rx = 0;
		HAL_UART_Receive_DMA(BtUartHandle, &bt_rx, 1 );
	}
	else if( huart->Instance == USART2 )
	{
		StarterUartRxCallback();
	}


}

int LoadBuffer(uint8_t* targetbuf)
{
	char buf [200];
	uint16_t ptr = 0;
	uint32_t size = 0;
	memcpy( buf + ptr, &size, sizeof(size) );
	ptr += sizeof(size);
	memcpy( buf + ptr, &DebugMode, sizeof(DebugMode) );
	ptr += sizeof(DebugMode);
	memcpy( buf + ptr, &Running, sizeof(Running) );
	ptr += sizeof(Running);
	memcpy( buf + ptr, &Mode, sizeof(Mode) );
	ptr += sizeof(Mode);
	memcpy( buf + ptr, &GyorsasagiState, sizeof(GyorsasagiState) );
	ptr += sizeof(GyorsasagiState);
	memcpy( buf + ptr, &UgyessegiState, sizeof(UgyessegiState) );
	ptr += sizeof(UgyessegiState);
	memcpy( buf + ptr, &Timestamp, sizeof(Timestamp) );
	ptr += sizeof(Timestamp);
	memcpy( buf + ptr, &TimeBetweenData,sizeof(TimeBetweenData) );
	ptr += sizeof(TimeBetweenData);
	memcpy( buf + ptr, &LinePos_controller, sizeof(LinePos_controller) );
	ptr += sizeof(LinePos_controller);
	memcpy( buf + ptr, &LinePositionBack, sizeof(LinePositionBack) );
	ptr += sizeof(LinePositionBack);
	memcpy( buf + ptr, &Vonalszog_controller, sizeof(Vonalszog_controller) );
	ptr += sizeof(Vonalszog_controller);
	memcpy( buf + ptr, &LineNumFront, sizeof(LineNumFront) );
	ptr += sizeof(LineNumFront);
	memcpy( buf + ptr, &LineNumBack, sizeof(LineNumBack) );
	ptr += sizeof(LineNumBack);
	memcpy( buf + ptr, &LinearX, sizeof(LinearX) );
	ptr += sizeof(LinearX);
	memcpy( buf + ptr, &LinearY, sizeof(LinearY) );
	ptr += sizeof(LinearY);
	memcpy( buf + ptr, &LinearZ, sizeof(LinearZ) );
	ptr += sizeof(LinearZ);
	memcpy( buf + ptr, &AngularX, sizeof(AngularX) );
	ptr += sizeof(AngularX);
	memcpy( buf + ptr, &AngularY, sizeof(AngularY) );
	ptr += sizeof(AngularY);
	memcpy( buf + ptr, &AngularZ, sizeof(AngularZ) );
	ptr += sizeof(AngularZ);
	memcpy( buf + ptr, &DistanceFront, sizeof(DistanceFront) );
	ptr += sizeof(DistanceFront);
	memcpy( buf + ptr, &DistanceLeft, sizeof(DistanceLeft) );
	ptr += sizeof(DistanceLeft);
	memcpy( buf + ptr, &DistanceRight, sizeof(DistanceRight) );
	ptr += sizeof(DistanceRight);
	memcpy( buf + ptr, &Speed, sizeof(Speed) );
	ptr += sizeof(Speed);
	memcpy( buf + ptr, &Uthossz, sizeof(Uthossz) );
	ptr += sizeof(Uthossz);
	memcpy( buf + ptr, &MotorCurrent, sizeof(MotorCurrent) );
	ptr += sizeof(MotorCurrent);
	memcpy( buf + ptr, &SpeedSP, sizeof(SpeedSP) );
	ptr += sizeof(SpeedSP);
	memcpy( buf + ptr, &TclMotor, sizeof(TclMotor) );
	ptr += sizeof(TclMotor);
	memcpy( buf + ptr, &SpeedSPGyors, sizeof(SpeedSPGyors) );
	ptr += sizeof(SpeedSPGyors);
	memcpy( buf + ptr, &SpeedSPKanyar, sizeof(SpeedSPKanyar) );
	ptr += sizeof(SpeedSPKanyar);

	memcpy( buf + ptr, &KpGyors,sizeof(KpGyors) );
	ptr += sizeof(KpGyors);
	memcpy( buf + ptr, &KpKanyar,sizeof(KpKanyar) );
	ptr += sizeof(KpKanyar);

	memcpy( buf + ptr, &KdGyors,sizeof(KdGyors) );
	ptr += sizeof(KdGyors);
	memcpy( buf + ptr, &KdKanyar,sizeof(KdKanyar) );
	ptr += sizeof(KdKanyar);

	memcpy( buf + ptr, &BatteryMotor, sizeof(BatteryMotor) );
	ptr += sizeof(BatteryMotor);
	memcpy( buf + ptr, &BatteryLogic, sizeof(BatteryLogic) );
	ptr += sizeof(BatteryLogic);

	memcpy( buf + ptr, &ServoPos, sizeof(ServoPos) );
	ptr += sizeof(ServoPos);

	memcpy( buf + ptr, &Kp,sizeof(Kp) );
	ptr += sizeof(Kp);
	memcpy( buf + ptr, &Ki,sizeof(Ki) );
	ptr += sizeof(Ki);
	memcpy( buf + ptr, &Kd,sizeof(Kd) );
	ptr += sizeof(Kd);
	memcpy( buf + ptr, &kszi,sizeof(kszi) );
	ptr += sizeof(kszi);
	memcpy( buf + ptr, &OutputDivisor,sizeof(OutputDivisor) );
	ptr += sizeof(OutputDivisor);
	memcpy( buf + ptr, &D5percent,sizeof(D5percent) );
	ptr += sizeof(D5percent);
	memcpy( buf + ptr, &D5Add,sizeof(D5Add) );
	ptr += sizeof(D5Add);
	memcpy( buf + ptr, &D5Mul,sizeof(D5Mul) );
	ptr += sizeof(D5Mul);
	memcpy( buf + ptr, &KpWeight,sizeof(KpWeight) );
	ptr += sizeof(KpWeight);
	memcpy( buf + ptr, &KdeltaWeight,sizeof(KdeltaWeight) );
	ptr += sizeof(KdeltaWeight);

	memcpy( buf + ptr, &KorforgalomData ,sizeof(KorforgalomData) );
	ptr += sizeof(KorforgalomData);
	size = ptr;

	memcpy( buf, &size, sizeof(size) );

	memcpy( targetbuf, buf, sizeof(buf) );

	return size;
}


