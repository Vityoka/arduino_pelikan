/*
 * register_map.c
 *
 *  Created on: Nov 7, 2017
 *      Author: smate
 */


#include "register_map.h"
#include "statemachine.h"

void InitRegisters(void)
{
	Mode = SLAVE;

	LinePositionFront = 0.0;
	LinePositionBack = 0.0;
	LineAngle = 0.0;
	LineNumFront = 0;
	LineNumBack = 0;
	SELFTEST = 0x00;
	DebugFirstRun = 0xFF;
	CalibRequestState = 0;

	BatteryLogic = 0;
	BatteryMotor = 0;

	DebugState = 0x00;
	DebugMode = 0x00;
	Running = RUN_STOP;
	Timestamp = 0x00;
	LinePositionFront = 4.4;
	LinePositionBack = 3.3;
	LineAngle = 2.1;
	LineNumFront = 0x00;
	LineNumBack = 0x00;
	ActiveInfraFront = 0;
	ActiveInfraBack = 0;
	LinearX = 0x00;
	LinearY = 0x00;
	LinearZ = 0x00;
	AngularX = 0x00;
	AngularY = 0x00;
	AngularZ = 0x00;
	DistanceFront = 0;
	DistanceLeft = 0;
	DistanceRight = 0;
	Speed = 0x00;
	MotorCurrent = 0;
	Uthossz = 0;
	UthosszTarget = 0;

	TclMotor = 200;

	kszi = 1.1;
	OutputDivisor = 1;
	D5percent = 2;
	D5Add = 0.1;
	D5Mul = 0.35;
	KpWeight = 1;
	KdeltaWeight = 44.44;

	SpeedSP = 0;
	SpeedSPGyors = 2;
	SpeedSPFek = 0;
	SpeedSPKanyar = 1.2;

	KpGyors=2.75;
	KpKanyar=9;
	KdGyors=0.6;
	KdKanyar=0;

	BatteryMotor = 0;
	BatteryLogic = 0;
	ServoPos = 0x00;
	Kp = KpKanyar;
	Ki = 1;		//Jelenleg nem használjuk
	Kd = KdKanyar;
	for(uint8_t i = 0; i < 64 ; i++)
	{
		LineInfra[i] = i%2;
	}

	//ugyessegi palya allapotgep
	UgyessegiState = DETECT_OBSTACLE;
	GyorsasagiState = 0 ;	//átmenetileg amig nemtudom a hipotézer brachet idemergelni.
	StarterReady = 0;
	StarterBegin = 0;
	DroneDetected = 0;
	KorforgalomData = 0;
	MovAvgFrontEnabled = 1;

	return;
}

