/*
 * register_map.h
 *
 *  Created on: Nov 7, 2017
 *      Author: smate
 */

#ifndef REGISTER_MAP_H_
#define REGISTER_MAP_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "remotecontrol.h"
#include "ir.h"
#include "rpi.h"

//DEFINES
#define GYORSASAGI 0
#define UGYESSEGI 1
#define SLAVE 2

#define BT_BUFF_RX_SIZE 128
#define BT_BUFF_TX_SIZE 1024
#define LINE_BUFF_RX_SIZE 128
#define LINE_BUFF_TX_SIZE 128
#define MEMS_BUFF_RX_SIZE 128
#define MEMS_BUFF_TX_SIZE 128

volatile uint8_t BT_CONNECTED;

volatile uint8_t bt_buffer_rx[BT_BUFF_RX_SIZE];
volatile uint8_t bt_buffer_tx[BT_BUFF_TX_SIZE];
volatile uint8_t line_buffer_rx[LINE_BUFF_RX_SIZE];
volatile uint8_t line_buffer_tx[LINE_BUFF_TX_SIZE];
volatile uint8_t rpi_rx[SPI_RX_SIZE];
volatile uint8_t IR_Data[14];

volatile uint8_t bt_rx;
volatile uint16_t rx_pointer;


volatile float BatteryLogic;
volatile float BatteryMotor;

//
volatile uint8_t DebugMode;
volatile uint8_t Running;
volatile uint8_t Mode;
volatile uint64_t Timestamp;
volatile uint32_t TimeBetweenData;
volatile float LinePos_controller;
volatile float Vonalszog_controller;
volatile float LinePositionFront;
volatile float LinePositionBack;
volatile float LineAngle;
volatile uint8_t ActiveInfraFront;
volatile uint8_t ActiveInfraBack;
volatile uint8_t LineNumFront;
volatile uint8_t LineNumBack;
volatile float LinearX;
volatile float LinearY;
volatile float LinearZ;
volatile float AngularX;
volatile float AngularY;
volatile float AngularZ;
volatile float DistanceFront;
volatile float DistanceLeft;
volatile float DistanceRight;

volatile float Speed;
volatile float SpeedSP;
volatile float Uthossz;
volatile float UthosszTarget;
volatile float MotorCurrent;

volatile float SpeedSPGyors;
volatile float SpeedSPFek;
volatile float SpeedSPKanyar;

volatile uint16_t TclMotor;

volatile float OutputDivisor;
volatile float kszi;
volatile float D5percent;
volatile float D5Mul;
volatile float D5Add;
volatile float KpWeight;
volatile float KdeltaWeight;

volatile float KpGyors;
volatile float KpKanyar;
volatile float KdGyors;
volatile float KdKanyar;

volatile float ServoPos;

volatile uint16_t LineInfra[64];

//Controller
//PID
volatile float Kp, Ki, Kd;

volatile uint64_t temp_reverse;

//FLAGS
volatile uint8_t LineDataReady;
volatile uint8_t LineDataWait;
volatile uint8_t SELFTEST;
volatile uint8_t DebugFirstRun;
volatile uint8_t DebugState;
volatile uint8_t CalibRequestState;


//ugyessegi palya allapotgep
volatile uint8_t UgyessegiState;
volatile uint8_t GyorsasagiState;

volatile uint8_t KorforgalomData;
volatile uint8_t StarterReady;
volatile uint8_t StarterBegin;
volatile uint8_t DroneDetected;
volatile uint8_t MovAvgFrontEnabled;

//functions

void InitRegisters(void);

#endif /* REGISTER_MAP_H_ */
