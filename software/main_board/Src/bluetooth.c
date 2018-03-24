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

extern UART_HandleTypeDef huart1;
uint8_t hmi_rx_index = 0;

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
			CommandParser();
			Timestamp++;
			TimeBetweenData = HAL_GetTick() - TimeElapsedFromStart;
			TimeElapsedFromStart = HAL_GetTick();
			if(DebugMode)
			{
				//LoadBufferDebug();
			}
			else
			{
				LoadBuffer();
			}

		}
		else
		{

		}
		osDelay(50);
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

void CommandParser(void)
{
	if(bt_buffer_rx[4] != 0)
	{
		uint32_t commandSize = 0;
		commandSize |= (bt_buffer_rx[0] << 24);
		commandSize |= (bt_buffer_rx[1] << 16);
		commandSize |= (bt_buffer_rx[2] << 8);
		commandSize |= bt_buffer_rx[3];
		if( rx_pointer < commandSize)
		{
			return;
		}
		else
		{
			switch(bt_buffer_rx[4])
			{
				case 'G':	// run
					Running = bt_buffer_rx[5];
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
					memcpy(&SpeedSP, (bt_buffer_rx + 5), 4);
					break;
				case 'T':	//timestamp
					Timestamp = 0;
					Timestamp |= bt_buffer_rx[5];
					Timestamp <<= 8;
					Timestamp |= bt_buffer_rx[6];
					Timestamp <<= 8;
					Timestamp |= bt_buffer_rx[7];
					Timestamp <<= 8;
					Timestamp |= bt_buffer_rx[8];
					Timestamp <<= 8;
					Timestamp |= bt_buffer_rx[9];
					Timestamp <<= 8;
					Timestamp |= bt_buffer_rx[10];
					Timestamp <<= 8;
					Timestamp |= bt_buffer_rx[11];
					Timestamp <<= 8;
					Timestamp |= bt_buffer_rx[12];
					break;
				case 'L':	//calibrate light
					CalibRequestState = 1;
					break;
				case 'D':	//calibrate dark
					break;
				case '1':	//Kp
					Kp = 0;
					memcpy(&Kp, (bt_buffer_rx + 5), 4);
					break;
				case '2':	//Ki
					Ki = 0;
					memcpy(&Ki, (bt_buffer_rx + 5), 4);
					break;
				case '3':	//Kd
					Kd = 0;
					memcpy(&Kd, (bt_buffer_rx + 5), 4);
					break;
				case 'q':	//KpGyors
					KpGyors = 0;
					memcpy(&KpGyors, (bt_buffer_rx + 5), 4);
					break;
				case 'w':	//KdGyors
					KdGyors = 0;
					memcpy(&KdGyors, (bt_buffer_rx + 5), 4);
					break;
				case 'e':	//KpKanyar
					KpKanyar = 0;
					memcpy(&KpKanyar, (bt_buffer_rx + 5), 4);
					break;
				case 'r':	//KdKanyar
					KdKanyar = 0;
					memcpy(&KdKanyar, (bt_buffer_rx + 5), 4);
					break;
				case 'X':	//selftest
					SELFTEST = 0xFF;
					break;
				case 'U':	//ügyességi mód
					Mode = UGYESSEGI;
					 SetTasks();
					break;
				case 'Y':	//gyorsasági mód
					Mode = GYORSASAGI;
					 SetTasks();
					break;
				case 'l':	//TclMotor
					TclMotor = 0;
					TclMotor |= (bt_buffer_rx[5] << 8);
					TclMotor |= bt_buffer_rx[6];
					break;
				case 'u':	//uthossz nullazasa
					Uthossz = 0;
					break;
				case 'y':	//ugyessegi állapotgép
					UgyessegiState = bt_buffer_rx[5];
					break;
				case 'g':	//gyorsasági állapotgép
					GyorsasagiState = bt_buffer_rx[5];
					break;
				case 'k':	//kszi
					kszi = 0;
					memcpy(&kszi, (bt_buffer_rx + 5), 4);
					break;
				case '*':	//D5Mul
					D5Mul = 0;
					memcpy(&D5Mul, (bt_buffer_rx + 5), 4);
					break;
				case '+':	//D5Add
					D5Add = 0;
					memcpy(&D5Add, (bt_buffer_rx + 5), 4);
					break;
				case 's':	//ServoPos
					ServoPos = 0;
					memcpy(&ServoPos, (bt_buffer_rx + 5), 4);
					break;
				case 'o':	//OutputDivisor
					OutputDivisor = 0;
					memcpy(&OutputDivisor, (bt_buffer_rx + 5), 4);
					break;
				case 'p':	//KpWeight
					KpWeight = 0;
					memcpy(&KpWeight, (bt_buffer_rx + 5), 4);
					break;
				case 'd':	//KdeltaWeight
					KdeltaWeight = 0;
					memcpy(&KdeltaWeight, (bt_buffer_rx + 5), 4);
					break;
				case '6':	//SpeedSPGyors
					SpeedSPGyors = 0;
					memcpy(&SpeedSPGyors, (bt_buffer_rx + 5), 4);
					break;
				case '7':	//SpeedSPKanyar
					SpeedSPKanyar = 0;
					memcpy(&SpeedSPKanyar, (bt_buffer_rx + 5), 4);
					break;





				default:
					return;
			}
			BluetoothFlushRX();
		}
	}
	return;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if( huart->Instance == UART4 )
	{
		HAL_UART_Receive_IT(&huart4, &hmi_buffer_rx[hmi_rx_index], 1 );
		hmi_rx_index++;
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
/*
void LoadBufferDebug(void)
{
	uint16_t ptr = 0;
	uint32_t size = 0;
	memcpy( bt_buffer_tx + ptr, &size, sizeof(size) );
	ptr += sizeof(size);
	memcpy( bt_buffer_tx + ptr, &DebugMode,sizeof(DebugMode) );
	ptr += sizeof(DebugMode);
	memcpy( bt_buffer_tx + ptr, &Running,sizeof(Running) );
	ptr += sizeof(Running);
	memcpy( bt_buffer_tx + ptr, &Timestamp,sizeof(Timestamp) );
	ptr += sizeof(Timestamp);

	memcpy( bt_buffer_tx + ptr, &LinePositionFront, sizeof(LinePositionFront) );
	ptr += sizeof(LinePositionFront);
	memcpy( bt_buffer_tx + ptr, &LinePositionBack,sizeof(LinePositionBack) );
	ptr += sizeof(LinePositionBack);
	memcpy( bt_buffer_tx + ptr, &LineAngle,sizeof(LineAngle) );
	ptr += sizeof(LineAngle);
	memcpy( bt_buffer_tx + ptr, &LineNumFront,sizeof(LineNumFront) );
	ptr += sizeof(LineNumFront);
	memcpy( bt_buffer_tx + ptr, &LineNumBack,sizeof(LineNumBack) );
	ptr += sizeof(LineNumBack);
	memcpy( bt_buffer_tx + ptr, &LinearX,sizeof(LinearX) );
	ptr += sizeof(LinearX);
	memcpy( bt_buffer_tx + ptr, &LinearY,sizeof(LinearY) );
	ptr += sizeof(LinearY);
	memcpy( bt_buffer_tx + ptr, &LinearZ,sizeof(LinearZ) );
	ptr += sizeof(LinearZ);
	memcpy( bt_buffer_tx + ptr, &AngularX,sizeof(AngularX) );
	ptr += sizeof(AngularX);
	memcpy( bt_buffer_tx + ptr, &AngularY,sizeof(AngularY) );
	ptr += sizeof(AngularY);
	memcpy( bt_buffer_tx + ptr, &AngularZ,sizeof(AngularZ) );
	ptr += sizeof(AngularZ);
	memcpy( bt_buffer_tx + ptr, &DistanceFront,sizeof(DistanceFront) );
	ptr += sizeof(DistanceFront);
	memcpy( bt_buffer_tx + ptr, &DistanceLeft,sizeof(DistanceLeft) );
	ptr += sizeof(DistanceLeft);
	memcpy( bt_buffer_tx + ptr, &DistanceRight,sizeof(DistanceRight) );
	ptr += sizeof(DistanceRight);
	memcpy( bt_buffer_tx + ptr, &Speed,sizeof(Speed) );
	ptr += sizeof(Speed);
	memcpy( bt_buffer_tx + ptr, &SpeedSP,sizeof(SpeedSP) );
	ptr += sizeof(SpeedSP);


	memcpy( bt_buffer_tx + ptr, &MotorCurrent,sizeof(MotorCurrent) );
	ptr += sizeof(MotorCurrent);
	memcpy( bt_buffer_tx + ptr, &BatteryMotor,sizeof(BatteryMotor) );
	ptr += sizeof(BatteryMotor);
	memcpy( bt_buffer_tx + ptr, &BatteryLogic,sizeof(BatteryLogic) );
	ptr += sizeof(BatteryLogic);
	memcpy( bt_buffer_tx + ptr, &ServoPos,sizeof(ServoPos) );
	ptr += sizeof(ServoPos);

	memcpy( bt_buffer_tx + ptr, &Kp,sizeof(Kp) );
	ptr += sizeof(Kp);
	memcpy( bt_buffer_tx + ptr, &Ki,sizeof(Ki) );
	ptr += sizeof(Ki);
	memcpy( bt_buffer_tx + ptr, &Kd,sizeof(Kd) );
	ptr += sizeof(Kd);

	memcpy( bt_buffer_tx + ptr, LineInfra, 128 );
	ptr += 128;
	size = ptr;
	memcpy( bt_buffer_tx, &size, sizeof(size) );
	BluetoothSend(size);
	return;
}
*/

void LoadBuffer(void)
{
	uint16_t ptr = 0;
	uint32_t size = 0;
	memcpy( bt_buffer_tx + ptr, &size, sizeof(size) );
	ptr += sizeof(size);
	memcpy( bt_buffer_tx + ptr, &DebugMode, sizeof(DebugMode) );
	ptr += sizeof(DebugMode);
	memcpy( bt_buffer_tx + ptr, &Running, sizeof(Running) );
	ptr += sizeof(Running);
	memcpy( bt_buffer_tx + ptr, &Mode, sizeof(Mode) );
	ptr += sizeof(Mode);
	memcpy( bt_buffer_tx + ptr, &GyorsasagiState, sizeof(GyorsasagiState) );
	ptr += sizeof(GyorsasagiState);
	memcpy( bt_buffer_tx + ptr, &UgyessegiState, sizeof(UgyessegiState) );
	ptr += sizeof(UgyessegiState);
	memcpy( bt_buffer_tx + ptr, &Timestamp, sizeof(Timestamp) );
	ptr += sizeof(Timestamp);
	memcpy( bt_buffer_tx + ptr, &TimeBetweenData,sizeof(TimeBetweenData) );
	ptr += sizeof(TimeBetweenData);
	memcpy( bt_buffer_tx + ptr, &LinePos_controller, sizeof(LinePos_controller) );
	ptr += sizeof(LinePos_controller);
	memcpy( bt_buffer_tx + ptr, &LinePositionBack, sizeof(LinePositionBack) );
	ptr += sizeof(LinePositionBack);
	memcpy( bt_buffer_tx + ptr, &Vonalszog_controller, sizeof(Vonalszog_controller) );
	ptr += sizeof(Vonalszog_controller);
	memcpy( bt_buffer_tx + ptr, &LineNumFront, sizeof(LineNumFront) );
	ptr += sizeof(LineNumFront);
	memcpy( bt_buffer_tx + ptr, &LineNumBack, sizeof(LineNumBack) );
	ptr += sizeof(LineNumBack);
	memcpy( bt_buffer_tx + ptr, &LinearX, sizeof(LinearX) );
	ptr += sizeof(LinearX);
	memcpy( bt_buffer_tx + ptr, &LinearY, sizeof(LinearY) );
	ptr += sizeof(LinearY);
	memcpy( bt_buffer_tx + ptr, &LinearZ, sizeof(LinearZ) );
	ptr += sizeof(LinearZ);
	memcpy( bt_buffer_tx + ptr, &AngularX, sizeof(AngularX) );
	ptr += sizeof(AngularX);
	memcpy( bt_buffer_tx + ptr, &AngularY, sizeof(AngularY) );
	ptr += sizeof(AngularY);
	memcpy( bt_buffer_tx + ptr, &AngularZ, sizeof(AngularZ) );
	ptr += sizeof(AngularZ);
	memcpy( bt_buffer_tx + ptr, &DistanceFront, sizeof(DistanceFront) );
	ptr += sizeof(DistanceFront);
	memcpy( bt_buffer_tx + ptr, &DistanceLeft, sizeof(DistanceLeft) );
	ptr += sizeof(DistanceLeft);
	memcpy( bt_buffer_tx + ptr, &DistanceRight, sizeof(DistanceRight) );
	ptr += sizeof(DistanceRight);
	memcpy( bt_buffer_tx + ptr, &Speed, sizeof(Speed) );
	ptr += sizeof(Speed);
	memcpy( bt_buffer_tx + ptr, &Uthossz, sizeof(Uthossz) );
	ptr += sizeof(Uthossz);
	memcpy( bt_buffer_tx + ptr, &MotorCurrent, sizeof(MotorCurrent) );
	ptr += sizeof(MotorCurrent);
	memcpy( bt_buffer_tx + ptr, &SpeedSP, sizeof(SpeedSP) );
	ptr += sizeof(SpeedSP);
	memcpy( bt_buffer_tx + ptr, &TclMotor, sizeof(TclMotor) );
	ptr += sizeof(TclMotor);
	memcpy( bt_buffer_tx + ptr, &SpeedSPGyors, sizeof(SpeedSPGyors) );
	ptr += sizeof(SpeedSPGyors);
	memcpy( bt_buffer_tx + ptr, &SpeedSPKanyar, sizeof(SpeedSPKanyar) );
	ptr += sizeof(SpeedSPKanyar);

	memcpy( bt_buffer_tx + ptr, &KpGyors,sizeof(KpGyors) );
	ptr += sizeof(KpGyors);
	memcpy( bt_buffer_tx + ptr, &KpKanyar,sizeof(KpKanyar) );
	ptr += sizeof(KpKanyar);

	memcpy( bt_buffer_tx + ptr, &KdGyors,sizeof(KdGyors) );
	ptr += sizeof(KdGyors);
	memcpy( bt_buffer_tx + ptr, &KdKanyar,sizeof(KdKanyar) );
	ptr += sizeof(KdKanyar);

	memcpy( bt_buffer_tx + ptr, &BatteryMotor, sizeof(BatteryMotor) );
	ptr += sizeof(BatteryMotor);
	memcpy( bt_buffer_tx + ptr, &BatteryLogic, sizeof(BatteryLogic) );
	ptr += sizeof(BatteryLogic);

	memcpy( bt_buffer_tx + ptr, &ServoPos, sizeof(ServoPos) );
	ptr += sizeof(ServoPos);

	memcpy( bt_buffer_tx + ptr, &Kp,sizeof(Kp) );
	ptr += sizeof(Kp);
	memcpy( bt_buffer_tx + ptr, &Ki,sizeof(Ki) );
	ptr += sizeof(Ki);
	memcpy( bt_buffer_tx + ptr, &Kd,sizeof(Kd) );
	ptr += sizeof(Kd);
	memcpy( bt_buffer_tx + ptr, &kszi,sizeof(kszi) );
	ptr += sizeof(kszi);
	memcpy( bt_buffer_tx + ptr, &OutputDivisor,sizeof(OutputDivisor) );
	ptr += sizeof(OutputDivisor);
	memcpy( bt_buffer_tx + ptr, &D5percent,sizeof(D5percent) );
	ptr += sizeof(D5percent);
	memcpy( bt_buffer_tx + ptr, &D5Add,sizeof(D5Add) );
	ptr += sizeof(D5Add);
	memcpy( bt_buffer_tx + ptr, &D5Mul,sizeof(D5Mul) );
	ptr += sizeof(D5Mul);
	memcpy( bt_buffer_tx + ptr, &KpWeight,sizeof(KpWeight) );
	ptr += sizeof(KpWeight);
	memcpy( bt_buffer_tx + ptr, &KdeltaWeight,sizeof(KdeltaWeight) );
	ptr += sizeof(KdeltaWeight);

	memcpy( bt_buffer_tx + ptr, &KorforgalomData ,sizeof(KorforgalomData) );
	ptr += sizeof(KorforgalomData);

	size = ptr;
	memcpy( bt_buffer_tx, &size, sizeof(size) );
	BluetoothSend(size);
	return;
}

void HMISend(uint16_t size)
{
	HAL_UART_Transmit_DMA(&huart4, hmi_buffer_tx, size);
	return;
}

void HMIReceive(void)
{
	hmi_rx_index = 0;
	HAL_UART_Receive_IT(&huart4, &hmi_buffer_rx[hmi_rx_index], 1 );
	return;
}
