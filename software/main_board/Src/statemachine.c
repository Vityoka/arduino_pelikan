/*
 * statemachine.c
 *
 *  Created on: 2018. jan. 15.
 *      Author: Vityó
 */

#include "statemachine.h"
#include "line_sensor.h"
#include "adc.h"
#include "timers.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

extern EventGroupHandle_t Eventgroup_Triggers;
extern osThreadId SteeringStateSpaceControllerHandle;
extern osThreadId RollAngleControllerTaskHandle;
extern osThreadId Roll;
extern SemaphoreHandle_t Semaphore_Uthossz_Achieved;
TimerHandle_t xTimers[NUM_OF_TIMERS];

int kijarat;
int negirany;

void StateMachineTask(){
	EventBits_t setBits;
	EventBits_t clearBits;
	while(1)
	{
		switch(UgyessegiState)
		{
		case DETECT_OBSTACLE:
			setBits = xEventGroupWaitBits(
								Eventgroup_Triggers ,
								BIT_STARTER | BIT_DRONE | BIT_KORFORGALOM | BIT_UTCASAROK | BIT_VASUTIATJARO | BIT_KONVOJ | BIT_CEL | BIT_FORGOHORDO ,
								pdTRUE,
								pdFALSE,
								portMAX_DELAY);
			if( ( setBits & BIT_STARTER ) != 0  )
			{
				UgyessegiState = STARTKAPU;
			}
			else if( ( setBits & BIT_DRONE ) != 0  )
			{
				UgyessegiState = DRONE;
			}
			else if( ( setBits & BIT_KORFORGALOM ) != 0  )
			{
				UgyessegiState = KORFORGALOM;
			}
			else if( ( setBits & BIT_UTCASAROK ) != 0  )
			{
				UgyessegiState = UTCASAROK;
			}
			else if( ( setBits & BIT_VASUTIATJARO ) != 0  )
			{
				UgyessegiState = VASUTIATJARO;
			}
			else if( ( setBits & BIT_KONVOJ ) != 0  )
			{
				UgyessegiState = KONVOJ;
			}
			else if( ( setBits & BIT_CEL ) != 0  )
			{
				UgyessegiState = CEL;
			}
			else if( ( setBits & BIT_FORGOHORDO ) != 0  )
			{
				UgyessegiState = FORGOHORDO;
			}
			break;


		case STARTKAPU:			//állapot
			//Állapotváltás feltétele
			if(StarterReady)
			{
				//Állapot vége
				UgyessegiState = STARTKAPU2;
				StarterReady = 0;

				//Következõ Állapot eleje
				Uthossz = 0;
				SpeedSP = 1;
			}
			break;

		case STARTKAPU2:
			//Állapotváltás feltétele
			if( Uthossz > 1)
			{
				//Állapot vége
				UgyessegiState = DETECT_OBSTACLE;
				clearBits = xEventGroupClearBits( Eventgroup_Triggers,
											      BIT_STARTER | BIT_DRONE | BIT_KORFORGALOM | BIT_UTCASAROK | BIT_VASUTIATJARO | BIT_KONVOJ | BIT_CEL | BIT_FORGOHORDO );
				//Következõ Állapot eleje
			}
			break;

		case DRONE:
			//Kezdeti sebesség
			SpeedSP = 0.5;
			//Állapotváltás feltétele
			if( DistanceFront < 60 )
			{
				//Állapot vége
				UgyessegiState = DRONE2;
				//Következõ Állapot eleje
				SpeedSP = 0;
			}
			break;
		case DRONE2:
			//Állapotváltás feltétele
			if( DistanceFront > 100 )
			{
				//Állapot vége
				UgyessegiState = DRONE3;

				//Következõ Állapot eleje
				osDelay(2000);
				Uthossz = 0;
				SpeedSP = 0.5;
			}
			break;
		case DRONE3:
			//Állapotváltás feltétele
			if( Uthossz > 1 )
			{
				//Állapot vége
				UgyessegiState = DETECT_OBSTACLE;
				clearBits = xEventGroupClearBits( Eventgroup_Triggers,
											   BIT_STARTER | BIT_DRONE | BIT_KORFORGALOM | BIT_UTCASAROK | BIT_VASUTIATJARO | BIT_KONVOJ | BIT_CEL | BIT_FORGOHORDO );
				//Következõ Állapot eleje
			}
			break;
		case KORFORGALOM:
			//Várakozás helyes adatra
			if(KorforgalomData)
			{
				//Hányadik kijárat
				kijarat = KorforgalomData % 3;
				negirany = KorforgalomData / 4;
				Uthossz = 0;
				UgyessegiState = KORFORGALOM_BEHAJTAS;
			}
			break;
		case KORFORGALOM_BEHAJTAS:
			//Automata vonalkövetés lekapcsolása
			osThreadSuspend(SteeringStateSpaceControllerHandle);

			//Indulás elõre
			SpeedSP = 0.5;

			//Szervo beállítás
			if (negirany)		//Negatív irányban kell körbemenni
				ServoPos = -1;	//Full jobbra
			else				//Pozitív irányban kell körbemenni
				ServoPos = 1;	//Full balra

			if(Uthossz > 0.65 * 3.14 / 3) //1/6 körív, 60°-ba befordulás
			{
				Uthossz = 0;
				UgyessegiState = KORFORGALOM_KORBEMENETEL;
			}
			break;

		case KORFORGALOM_KORBEMENETEL:
			if (negirany)		//Balra kanyarodunk
				ServoPos = 0.4;	//Erõsen balra
			else				//Jobbra kanyarodunk
				ServoPos = -0.4;//Erõsen jobbra

			if(Uthossz > (0.75 *3.14 /2 * kijarat - 0.65450)) //1/4 körív * kijárat - 2* 25°-os ívhossz
			{
				Uthossz = 0;
				UgyessegiState = KORFORGALOM_KIHAJTAS;
			}
			break;

		case KORFORGALOM_KIHAJTAS:
			if (negirany)		//Jobbra kihajtunk
				ServoPos = -1;	//Full jobb
			else				//Balra kihajtunk
				ServoPos = 1;	//Full bal

			if(LineNumFront > 0)
			{
				//Simán megtaláltuk a vonalat
				osThreadResume(SteeringStateSpaceControllerHandle);
				UgyessegiState = DETECT_OBSTACLE;
				clearBits = xEventGroupClearBits( Eventgroup_Triggers,
												  BIT_STARTER | BIT_DRONE | BIT_KORFORGALOM | BIT_UTCASAROK | BIT_VASUTIATJARO | BIT_KONVOJ | BIT_CEL | BIT_FORGOHORDO );

			}
			if(Uthossz > 0.65 * 3.14 / 3) //1/6 körív, 60°-ba befordulás
			{
				//Elvileg kéne vonalnak lenni, de nincs, úgyhogy megyünk egyenesen hátha lesz vonal
				osThreadResume(SteeringStateSpaceControllerHandle);
				UgyessegiState = KORFORGALOM_VONALKERESES;
			}
			break;
		case KORFORGALOM_VONALKERESES:
			ServoPos = 0;
			if(LineNumFront > 0)
			{
				//Megvan a vonal
				osThreadResume(SteeringStateSpaceControllerHandle);
				UgyessegiState = DETECT_OBSTACLE;
				clearBits = xEventGroupClearBits( Eventgroup_Triggers,
												  BIT_STARTER | BIT_DRONE | BIT_KORFORGALOM | BIT_UTCASAROK | BIT_VASUTIATJARO | BIT_KONVOJ | BIT_CEL | BIT_FORGOHORDO );
			}
			break;
		case UTCASAROK:			//addig megyünk amig elöl nem látunk már vonalat
			//Kezdeti sebesség
			SpeedSP = 0.5;
			//Állapotváltás feltétele
			if( LineNumFront == 0 )
			{
				//Állapot vége
				UgyessegiState = UTCASAROK_ATHAJTAS_SZEMBE;

				//Következõ Állapot eleje
				osThreadSuspend(SteeringStateSpaceControllerHandle);	//vonalkövetést kikapcsoljuk
				Uthossz = 0;
			}
			break;
		case UTCASAROK_ATHAJTAS_SZEMBE:		//elõremegyünk egy adott szakasznyit hogy érzékeljük meliyk irányban van a fal.
			//Állapotváltás feltétele
			if( Uthossz > 1 )
			{
				//Állapot vége
				UgyessegiState = UTCASAROK_FALIRANY_DETECT;

				//Következõ Állapot eleje
				SpeedSP = 0;
				osDelay(500);		//várunk amig megáll a kocsi.
			}
			break;
		case UTCASAROK_FALIRANY_DETECT:		//Érzékeljük melyik irányban van a fal
			//Állapotváltás feltétele
			if( DistanceLeft < 25.0  ) //ha bal oldalon fal van
			{
				//Állapot vége
				UgyessegiState = UTCASAROK_BALFAL_ELORE;
				//Következõ Állapot eleje
				Uthossz = 0;
				SpeedSP = 0.3;

			}
			//Állapotváltás feltétele
			if( DistanceRight < 25.0  ) //ha jobb oldalon fal van
			{
				//Állapot vége
				UgyessegiState = UTCASAROK_JOBBFAL_ELORE;
				//Következõ Állapot eleje
				SpeedSP = 0.3;
				Uthossz = 0;
			}

			break;
		case UTCASAROK_BALFAL_ELORE:		//Balra volt a fal, elõremegyünk még egy kicsit a tolatáshoz, és beállítjuk a kormányzást jobbra.
			//Állapotváltás feltétele
			if( Uthossz > 0.3 )
			{
				//Állapot vége
				UgyessegiState = UTCASAROK_TOLATAS;

				//Következõ Állapot eleje
				SpeedSP = 0;
				ServoPos = -0.3;
				osDelay(500);			//várunk amig megáll a kocsi.
				Uthossz = 0;
				SpeedSP = -0.3;
			}
			break;
		case UTCASAROK_JOBBFAL_ELORE:		//Jobbra volt a fal, elõremegyünk még egy kicsit a tolatáshoz, és beállítjuk a kormányzást balra.
			//Állapotváltás feltétele
			if( Uthossz > 0.3 )
			{
				//Állapot vége
				UgyessegiState = UTCASAROK_TOLATAS;

				//Következõ Állapot eleje
				SpeedSP = 0;
				ServoPos = 0.3;
				osDelay(500);			//várunk amig megáll a kocsi.
				Uthossz = 0;
				SpeedSP = -0.3;
			}
			break;
		case UTCASAROK_TOLATAS:				//Tolatunk egy negyed körívnyit
			//Állapotváltás feltétele
			if( Uthossz < -1.57 )		//kör sugara
			{
				//Állapot vége
				UgyessegiState = UTCASAROK_UJRAELINDULAS;

				//Következõ Állapot eleje
				SpeedSP = 0;
				osDelay(500);			//várunk amig megáll a kocsi.
				ServoPos = 0;			//kiegyenesítük a kormányzást
				osDelay(500);
				SpeedSP = 0.3;			//Elindulunk
				Uthossz = 0;
			}
			break;
		case UTCASAROK_UJRAELINDULAS:
			//Állapotváltás feltétele
			if( Uthossz > 0.57 )		//várunk hogy annyit menjünk hogy ne lássunk zavaró dupla vonalakat.
			{
				//Állapot vége
				UgyessegiState = UTCASAROK_VONALVARAS;

				//Következõ Állapot eleje

			}
			break;
		case UTCASAROK_VONALVARAS:
			//Állapotváltás feltétele
			if( LineNumFront != 0 )		//várunk hogy újra lássunk vonalat.
			{
				//Állapot vége
				UgyessegiState = UTCASAROK_KIHAJTAS;

				//Következõ Állapot eleje
				osThreadResume(SteeringStateSpaceControllerHandle);	//vonalkövetést visszakapcsoljuk
				Uthossz = 0;
			}
			break;
		case UTCASAROK_KIHAJTAS:				//megyünk elõre egy métert hogy kiérjünk az utcasarokból
			//Állapotváltás feltétele
			if( Uthossz > 1 )		//várunk hogy újra lássunk vonalat.
			{
				//Állapot vége
				UgyessegiState = DETECT_OBSTACLE;
				clearBits = xEventGroupClearBits( Eventgroup_Triggers,
											   BIT_STARTER | BIT_DRONE | BIT_KORFORGALOM | BIT_UTCASAROK | BIT_VASUTIATJARO | BIT_KONVOJ | BIT_CEL | BIT_FORGOHORDO );
			}
			break;
		case KONVOJ:
			if(1 )
			{

			}
			else
			{

			}
			break;
		case VASUTIATJARO:
			//Állapotváltás feltétele
			SpeedSP = 0.5;
			//Állapot vége
			UgyessegiState = VASUTIATJARO2;

			//Következõ Állapot eleje
			Uthossz = 0;
			break;
		case VASUTIATJARO2:
			//Állapotváltás feltétele
			if( Uthossz > 0.3 )
			{
				//Állapot vége
				UgyessegiState = VASUTIATJARO3;
				//Következõ Állapot eleje
				SpeedSP = 0;
			}
			break;
		case VASUTIATJARO3:
			//Állapotváltás feltétele
			if( DistanceFront < 60 )		//várunk hogy elmenjen egy kocsi
			{
				//Állapot vége
				UgyessegiState = VASUTIATJARO4;
				//Következõ Állapot eleje
				if( xTimerStart( xTimers[0] , 0) != pdPASS)
				{
					asm("bkpt");
				}
			}
			break;
		case VASUTIATJARO4:

			if( DistanceFront < 60 )		//várunk hogy elmenjen egy kocsi
			{
				if( xTimerStart( xTimers[0] , 0) != pdPASS)
				{
					asm("bkpt");
				}
			}
			//Állapotváltás feltétele
			if( CarCounterTimerElapsedFlag == 1 )		//várunk hogy lejárjon a timer
			{
				CarCounterTimerElapsedFlag = 0;
				//Állapot vége
				UgyessegiState = VASUTIATJARO5;
				//Következõ Állapot eleje
				SpeedSP = 0.5;
				Uthossz = 0;
			}
			break;
		case VASUTIATJARO5:
			//Állapotváltás feltétele
			if( Uthossz > 0.5 )
			{
				//Állapot vége
				UgyessegiState = VASUTIATJARO6;
				//Következõ Állapot eleje
				SpeedSP = 0;
			}
			break;
		case VASUTIATJARO6:
		//Állapotváltás feltétele
		if( DistanceFront < 60 )		//várunk hogy elmenjen egy kocsi
		{
			//Állapot vége
			UgyessegiState = VASUTIATJARO7;
			//Következõ Állapot eleje
			if( xTimerStart( xTimers[0] , 0) != pdPASS)
			{
				asm("bkpt");
			}
		}
		break;
		case VASUTIATJARO7:

			if( DistanceFront < 60 )		//várunk hogy elmenjen egy kocsi
			{
				if( xTimerStart( xTimers[0] , 0) != pdPASS)
				{
					asm("bkpt");
				}
			}
			//Állapotváltás feltétele
			if( CarCounterTimerElapsedFlag == 1 )		//várunk hogy lejárjon a timer
			{
				CarCounterTimerElapsedFlag = 0;
				//Állapot vége
				UgyessegiState = VASUTIATJARO8;
				//Következõ Állapot eleje
				SpeedSP = 1;
				Uthossz = 0;
			}
			break;
		case VASUTIATJARO8:
			//Állapotváltás feltétele
			if( Uthossz > 1 )
			{
				//Állapot vége
				UgyessegiState = DETECT_OBSTACLE;
				clearBits = xEventGroupClearBits( Eventgroup_Triggers,
												  BIT_STARTER | BIT_DRONE | BIT_KORFORGALOM | BIT_UTCASAROK | BIT_VASUTIATJARO | BIT_KONVOJ | BIT_CEL | BIT_FORGOHORDO );
			}
			break;
		case CEL:

			//Következõ Állapot eleje
			Uthossz = 0;
			UgyessegiState = CEL2;

			break;
		case CEL2:
			//Állapotváltás feltétele
			if( Uthossz > 0.4)
			{
				//Állapot vége
				UgyessegiState = DETECT_OBSTACLE;
				clearBits = xEventGroupClearBits( Eventgroup_Triggers,
												   BIT_STARTER | BIT_DRONE | BIT_KORFORGALOM | BIT_UTCASAROK | BIT_VASUTIATJARO | BIT_KONVOJ | BIT_CEL | BIT_FORGOHORDO );
				//Következõ Állapot eleje
				SpeedSP = 0;
			}
			break;
		case FORGOHORDO:
			//Állapot vége
			UgyessegiState = FORGOHORDO_RAMPA_FEL;

			//Következõ Állapot eleje
			SpeedSP = 0.5;

			break;
		case FORGOHORDO_RAMPA_FEL:
			//Állapotváltás feltétele
			if( AngularY < 3 && LineNumFront == 0 )
			{
				//Állapot vége
				UgyessegiState = FORGOHORDO_HORDOBAN;
				//Következõ Állapot eleje
				SpeedSP = 0.5;
				osThreadSuspend(SteeringStateSpaceControllerHandle);
				osThreadResume(RollAngleControllerTaskHandle);
			}
			break;
		case FORGOHORDO_HORDOBAN:
			//Állapotváltás feltétele
			if( LineNumFront == 1 )
			{
				//Állapot vége
				UgyessegiState = FORGOHORDO_KI;
				//Következõ Állapot eleje
				SpeedSP = 0.5;
				osThreadSuspend(RollAngleControllerTaskHandle);
				osThreadResume(SteeringStateSpaceControllerHandle);
				Uthossz = 0 ;
			}
			break;
		case FORGOHORDO_KI:
			//Állapotváltás feltétele
			if( Uthossz > 1 )
			{
				//Állapot vége
				UgyessegiState = DETECT_OBSTACLE;
				clearBits = xEventGroupClearBits( Eventgroup_Triggers,
												  BIT_STARTER | BIT_DRONE | BIT_KORFORGALOM | BIT_UTCASAROK | BIT_VASUTIATJARO | BIT_KONVOJ | BIT_CEL | BIT_FORGOHORDO );
			}
			break;
		}
		osDelay(100);
	}
	osThreadTerminate(NULL);
}

