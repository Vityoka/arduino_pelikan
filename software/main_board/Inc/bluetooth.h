/*
 * bluetooth.h
 *
 *  Created on: Nov 7, 2017
 *      Author: smate
 */

#ifndef BLUETOOTH_H_
#define BLUETOOTH_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "register_map.h"
#include "remotecontrol.h"

extern UART_HandleTypeDef huart4;

void BluetoothInit(UART_HandleTypeDef * UartHandle);
void BluetoothSend(uint16_t size);
void BluetoothReceive(void);
void BluetoothFlushTX(void);
void BluetoothFlushRX(void);
void CommandParser(uint8_t* rxbuf, uint16_t rx_index );
void LoadBufferDebug(void);
int LoadBuffer(uint8_t* targetbuf);
void BluetoothResetCallback();

UART_HandleTypeDef * BtUartHandle;

#endif /* BLUETOOTH_H_ */
