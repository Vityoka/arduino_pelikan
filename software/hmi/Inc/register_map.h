/*
 * register_map.h
 *
 *  Created on: Nov 7, 2017
 *      Author: smate
 */

#ifndef REGISTER_MAP_H_
#define REGISTER_MAP_H_

#include "inttypes.h"

//DEFINES
#define GYORSASAGI 0
#define UGYESSEGI 1
#define RUN_STOP			1	//kocsi megáll
#define RUN_FULL_AUTO				2	//kocsi gyorsulási módban,szabályzó futtatja
#define RUN_FULL_MANUAL			3	//kocsi a távirányítóval irányítható
#define RUN_MANUAL_THROTTLE		4	//gázt a távirányítóval adhatunk, de a kanyarodást a szabályzó végzi
#define RUN_MANUAL_STEERING	5	//gázt az alkalmazással adhatunk, de a kanyarodást a távirányító végzi

#define DETECT_OBSTACLE 0
#define STARTKAPU 10
#define STARTKAPU2 11
#define STARTKAPU3 12
#define DRONE 20
#define DRONE2 21
#define DRONE3 22
#define DRONE4 23
#define DRONE5 24
#define DRONE6 25
#define KORFORGALOM 30
#define KORFORGALOM_BEHAJTAS 31
#define KORFORGALOM_KORBEMENETEL 32
#define KORFORGALOM_KIHAJTAS 33
#define KORFORGALOM_VONALKERESES 34
#define UTCASAROK 40
#define UTCASAROK_ATHAJTAS_SZEMBE 41
#define UTCASAROK_FALIRANY_DETECT 42
#define UTCASAROK_BALFAL_ELORE 43
#define UTCASAROK_JOBBFAL_ELORE 44
#define UTCASAROK_TOLATAS 45
#define UTCASAROK_UJRAELINDULAS 46
#define UTCASAROK_VONALVARAS 47
#define UTCASAROK_KIHAJTAS 48
#define KONVOJ 50
#define VASUTIATJARO 60
#define VASUTIATJARO2 61
#define VASUTIATJARO3 62
#define VASUTIATJARO4 63
#define VASUTIATJARO5 64
#define VASUTIATJARO6 65
#define VASUTIATJARO7 66
#define VASUTIATJARO8 67
#define VASUTIATJARO9 68
#define FORGOHORDO 				70
#define FORGOHORDO_RAMPA_FEL	71
#define FORGOHORDO_HORDOBAN 	72
#define FORGOHORDO_KI 			73
#define CEL 80
#define CEL2 81

volatile float BatteryLogic;
volatile float BatteryMotor;
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

//Controller
//PID
volatile float Kp, Ki, Kd;

volatile uint64_t temp_reverse;



//ugyessegi palya allapotgep
volatile uint8_t UgyessegiState;
volatile uint8_t GyorsasagiState;

volatile uint8_t KorforgalomData;
volatile uint8_t StarterReady;
volatile uint8_t StarterBegin;
volatile uint8_t DroneDetected;
volatile uint8_t MovAvgFrontEnabled;

//kijelzõn megjelenõ sztringek
char str_mode [10];
char str_control [10];
char str_state_gyors [10];
char str_state_ugyes [10];

//functions

void InitRegisters(void);

#endif /* REGISTER_MAP_H_ */
