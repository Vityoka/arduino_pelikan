/*
 * statemachine.h
 *
 *  Created on: 2018. jan. 15.
 *      Author: Vityó
 */

#ifndef STATEMACHINE_H_
#define STATEMACHINE_H_

#include "register_map.h"

#define BIT_STARTER (1 << 0)
#define BIT_DRONE (1 << 1)
#define BIT_KORFORGALOM (1 << 2)
#define BIT_UTCASAROK (1 << 3)
#define BIT_VASUTIATJARO (1 << 4)
#define BIT_KONVOJ (1 << 5)
#define BIT_CEL (1 << 6)
#define BIT_FORGOHORDO (1 << 7)

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

int KonvojDir;
int CarCounterTimerElapsedFlag;

void detectObstacle();

#endif /* STATEMACHINE_H_ */
