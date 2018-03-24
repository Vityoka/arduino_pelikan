/*
 * statemachine.c
 *
 *  Created on: 2018. jan. 15.
 *      Author: Vity�
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


		case STARTKAPU:			//�llapot
			//�llapotv�lt�s felt�tele
			if(StarterReady)
			{
				//�llapot v�ge
				UgyessegiState = STARTKAPU2;
				StarterReady = 0;

				//K�vetkez� �llapot eleje
				Uthossz = 0;
				SpeedSP = 1;
			}
			break;

		case STARTKAPU2:
			//�llapotv�lt�s felt�tele
			if( Uthossz > 1)
			{
				//�llapot v�ge
				UgyessegiState = DETECT_OBSTACLE;
				clearBits = xEventGroupClearBits( Eventgroup_Triggers,
											      BIT_STARTER | BIT_DRONE | BIT_KORFORGALOM | BIT_UTCASAROK | BIT_VASUTIATJARO | BIT_KONVOJ | BIT_CEL | BIT_FORGOHORDO );
				//K�vetkez� �llapot eleje
			}
			break;

		case DRONE:
			//Kezdeti sebess�g
			SpeedSP = 0.5;
			//�llapotv�lt�s felt�tele
			if( DistanceFront < 60 )
			{
				//�llapot v�ge
				UgyessegiState = DRONE2;
				//K�vetkez� �llapot eleje
				SpeedSP = 0;
			}
			break;
		case DRONE2:
			//�llapotv�lt�s felt�tele
			if( DistanceFront > 100 )
			{
				//�llapot v�ge
				UgyessegiState = DRONE3;

				//K�vetkez� �llapot eleje
				osDelay(2000);
				Uthossz = 0;
				SpeedSP = 0.5;
			}
			break;
		case DRONE3:
			//�llapotv�lt�s felt�tele
			if( Uthossz > 1 )
			{
				//�llapot v�ge
				UgyessegiState = DETECT_OBSTACLE;
				clearBits = xEventGroupClearBits( Eventgroup_Triggers,
											   BIT_STARTER | BIT_DRONE | BIT_KORFORGALOM | BIT_UTCASAROK | BIT_VASUTIATJARO | BIT_KONVOJ | BIT_CEL | BIT_FORGOHORDO );
				//K�vetkez� �llapot eleje
			}
			break;
		case KORFORGALOM:
			//V�rakoz�s helyes adatra
			if(KorforgalomData)
			{
				//H�nyadik kij�rat
				kijarat = KorforgalomData % 3;
				negirany = KorforgalomData / 4;
				Uthossz = 0;
				UgyessegiState = KORFORGALOM_BEHAJTAS;
			}
			break;
		case KORFORGALOM_BEHAJTAS:
			//Automata vonalk�vet�s lekapcsol�sa
			osThreadSuspend(SteeringStateSpaceControllerHandle);

			//Indul�s el�re
			SpeedSP = 0.5;

			//Szervo be�ll�t�s
			if (negirany)		//Negat�v ir�nyban kell k�rbemenni
				ServoPos = -1;	//Full jobbra
			else				//Pozit�v ir�nyban kell k�rbemenni
				ServoPos = 1;	//Full balra

			if(Uthossz > 0.65 * 3.14 / 3) //1/6 k�r�v, 60�-ba befordul�s
			{
				Uthossz = 0;
				UgyessegiState = KORFORGALOM_KORBEMENETEL;
			}
			break;

		case KORFORGALOM_KORBEMENETEL:
			if (negirany)		//Balra kanyarodunk
				ServoPos = 0.4;	//Er�sen balra
			else				//Jobbra kanyarodunk
				ServoPos = -0.4;//Er�sen jobbra

			if(Uthossz > (0.75 *3.14 /2 * kijarat - 0.65450)) //1/4 k�r�v * kij�rat - 2* 25�-os �vhossz
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
				//Sim�n megtal�ltuk a vonalat
				osThreadResume(SteeringStateSpaceControllerHandle);
				UgyessegiState = DETECT_OBSTACLE;
				clearBits = xEventGroupClearBits( Eventgroup_Triggers,
												  BIT_STARTER | BIT_DRONE | BIT_KORFORGALOM | BIT_UTCASAROK | BIT_VASUTIATJARO | BIT_KONVOJ | BIT_CEL | BIT_FORGOHORDO );

			}
			if(Uthossz > 0.65 * 3.14 / 3) //1/6 k�r�v, 60�-ba befordul�s
			{
				//Elvileg k�ne vonalnak lenni, de nincs, �gyhogy megy�nk egyenesen h�tha lesz vonal
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
		case UTCASAROK:			//addig megy�nk amig el�l nem l�tunk m�r vonalat
			//Kezdeti sebess�g
			SpeedSP = 0.5;
			//�llapotv�lt�s felt�tele
			if( LineNumFront == 0 )
			{
				//�llapot v�ge
				UgyessegiState = UTCASAROK_ATHAJTAS_SZEMBE;

				//K�vetkez� �llapot eleje
				osThreadSuspend(SteeringStateSpaceControllerHandle);	//vonalk�vet�st kikapcsoljuk
				Uthossz = 0;
			}
			break;
		case UTCASAROK_ATHAJTAS_SZEMBE:		//el�remegy�nk egy adott szakasznyit hogy �rz�kelj�k meliyk ir�nyban van a fal.
			//�llapotv�lt�s felt�tele
			if( Uthossz > 1 )
			{
				//�llapot v�ge
				UgyessegiState = UTCASAROK_FALIRANY_DETECT;

				//K�vetkez� �llapot eleje
				SpeedSP = 0;
				osDelay(500);		//v�runk amig meg�ll a kocsi.
			}
			break;
		case UTCASAROK_FALIRANY_DETECT:		//�rz�kelj�k melyik ir�nyban van a fal
			//�llapotv�lt�s felt�tele
			if( DistanceLeft < 25.0  ) //ha bal oldalon fal van
			{
				//�llapot v�ge
				UgyessegiState = UTCASAROK_BALFAL_ELORE;
				//K�vetkez� �llapot eleje
				Uthossz = 0;
				SpeedSP = 0.3;

			}
			//�llapotv�lt�s felt�tele
			if( DistanceRight < 25.0  ) //ha jobb oldalon fal van
			{
				//�llapot v�ge
				UgyessegiState = UTCASAROK_JOBBFAL_ELORE;
				//K�vetkez� �llapot eleje
				SpeedSP = 0.3;
				Uthossz = 0;
			}

			break;
		case UTCASAROK_BALFAL_ELORE:		//Balra volt a fal, el�remegy�nk m�g egy kicsit a tolat�shoz, �s be�ll�tjuk a korm�nyz�st jobbra.
			//�llapotv�lt�s felt�tele
			if( Uthossz > 0.3 )
			{
				//�llapot v�ge
				UgyessegiState = UTCASAROK_TOLATAS;

				//K�vetkez� �llapot eleje
				SpeedSP = 0;
				ServoPos = -0.3;
				osDelay(500);			//v�runk amig meg�ll a kocsi.
				Uthossz = 0;
				SpeedSP = -0.3;
			}
			break;
		case UTCASAROK_JOBBFAL_ELORE:		//Jobbra volt a fal, el�remegy�nk m�g egy kicsit a tolat�shoz, �s be�ll�tjuk a korm�nyz�st balra.
			//�llapotv�lt�s felt�tele
			if( Uthossz > 0.3 )
			{
				//�llapot v�ge
				UgyessegiState = UTCASAROK_TOLATAS;

				//K�vetkez� �llapot eleje
				SpeedSP = 0;
				ServoPos = 0.3;
				osDelay(500);			//v�runk amig meg�ll a kocsi.
				Uthossz = 0;
				SpeedSP = -0.3;
			}
			break;
		case UTCASAROK_TOLATAS:				//Tolatunk egy negyed k�r�vnyit
			//�llapotv�lt�s felt�tele
			if( Uthossz < -1.57 )		//k�r sugara
			{
				//�llapot v�ge
				UgyessegiState = UTCASAROK_UJRAELINDULAS;

				//K�vetkez� �llapot eleje
				SpeedSP = 0;
				osDelay(500);			//v�runk amig meg�ll a kocsi.
				ServoPos = 0;			//kiegyenes�t�k a korm�nyz�st
				osDelay(500);
				SpeedSP = 0.3;			//Elindulunk
				Uthossz = 0;
			}
			break;
		case UTCASAROK_UJRAELINDULAS:
			//�llapotv�lt�s felt�tele
			if( Uthossz > 0.57 )		//v�runk hogy annyit menj�nk hogy ne l�ssunk zavar� dupla vonalakat.
			{
				//�llapot v�ge
				UgyessegiState = UTCASAROK_VONALVARAS;

				//K�vetkez� �llapot eleje

			}
			break;
		case UTCASAROK_VONALVARAS:
			//�llapotv�lt�s felt�tele
			if( LineNumFront != 0 )		//v�runk hogy �jra l�ssunk vonalat.
			{
				//�llapot v�ge
				UgyessegiState = UTCASAROK_KIHAJTAS;

				//K�vetkez� �llapot eleje
				osThreadResume(SteeringStateSpaceControllerHandle);	//vonalk�vet�st visszakapcsoljuk
				Uthossz = 0;
			}
			break;
		case UTCASAROK_KIHAJTAS:				//megy�nk el�re egy m�tert hogy ki�rj�nk az utcasarokb�l
			//�llapotv�lt�s felt�tele
			if( Uthossz > 1 )		//v�runk hogy �jra l�ssunk vonalat.
			{
				//�llapot v�ge
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
			//�llapotv�lt�s felt�tele
			SpeedSP = 0.5;
			//�llapot v�ge
			UgyessegiState = VASUTIATJARO2;

			//K�vetkez� �llapot eleje
			Uthossz = 0;
			break;
		case VASUTIATJARO2:
			//�llapotv�lt�s felt�tele
			if( Uthossz > 0.3 )
			{
				//�llapot v�ge
				UgyessegiState = VASUTIATJARO3;
				//K�vetkez� �llapot eleje
				SpeedSP = 0;
			}
			break;
		case VASUTIATJARO3:
			//�llapotv�lt�s felt�tele
			if( DistanceFront < 60 )		//v�runk hogy elmenjen egy kocsi
			{
				//�llapot v�ge
				UgyessegiState = VASUTIATJARO4;
				//K�vetkez� �llapot eleje
				if( xTimerStart( xTimers[0] , 0) != pdPASS)
				{
					asm("bkpt");
				}
			}
			break;
		case VASUTIATJARO4:

			if( DistanceFront < 60 )		//v�runk hogy elmenjen egy kocsi
			{
				if( xTimerStart( xTimers[0] , 0) != pdPASS)
				{
					asm("bkpt");
				}
			}
			//�llapotv�lt�s felt�tele
			if( CarCounterTimerElapsedFlag == 1 )		//v�runk hogy lej�rjon a timer
			{
				CarCounterTimerElapsedFlag = 0;
				//�llapot v�ge
				UgyessegiState = VASUTIATJARO5;
				//K�vetkez� �llapot eleje
				SpeedSP = 0.5;
				Uthossz = 0;
			}
			break;
		case VASUTIATJARO5:
			//�llapotv�lt�s felt�tele
			if( Uthossz > 0.5 )
			{
				//�llapot v�ge
				UgyessegiState = VASUTIATJARO6;
				//K�vetkez� �llapot eleje
				SpeedSP = 0;
			}
			break;
		case VASUTIATJARO6:
		//�llapotv�lt�s felt�tele
		if( DistanceFront < 60 )		//v�runk hogy elmenjen egy kocsi
		{
			//�llapot v�ge
			UgyessegiState = VASUTIATJARO7;
			//K�vetkez� �llapot eleje
			if( xTimerStart( xTimers[0] , 0) != pdPASS)
			{
				asm("bkpt");
			}
		}
		break;
		case VASUTIATJARO7:

			if( DistanceFront < 60 )		//v�runk hogy elmenjen egy kocsi
			{
				if( xTimerStart( xTimers[0] , 0) != pdPASS)
				{
					asm("bkpt");
				}
			}
			//�llapotv�lt�s felt�tele
			if( CarCounterTimerElapsedFlag == 1 )		//v�runk hogy lej�rjon a timer
			{
				CarCounterTimerElapsedFlag = 0;
				//�llapot v�ge
				UgyessegiState = VASUTIATJARO8;
				//K�vetkez� �llapot eleje
				SpeedSP = 1;
				Uthossz = 0;
			}
			break;
		case VASUTIATJARO8:
			//�llapotv�lt�s felt�tele
			if( Uthossz > 1 )
			{
				//�llapot v�ge
				UgyessegiState = DETECT_OBSTACLE;
				clearBits = xEventGroupClearBits( Eventgroup_Triggers,
												  BIT_STARTER | BIT_DRONE | BIT_KORFORGALOM | BIT_UTCASAROK | BIT_VASUTIATJARO | BIT_KONVOJ | BIT_CEL | BIT_FORGOHORDO );
			}
			break;
		case CEL:

			//K�vetkez� �llapot eleje
			Uthossz = 0;
			UgyessegiState = CEL2;

			break;
		case CEL2:
			//�llapotv�lt�s felt�tele
			if( Uthossz > 0.4)
			{
				//�llapot v�ge
				UgyessegiState = DETECT_OBSTACLE;
				clearBits = xEventGroupClearBits( Eventgroup_Triggers,
												   BIT_STARTER | BIT_DRONE | BIT_KORFORGALOM | BIT_UTCASAROK | BIT_VASUTIATJARO | BIT_KONVOJ | BIT_CEL | BIT_FORGOHORDO );
				//K�vetkez� �llapot eleje
				SpeedSP = 0;
			}
			break;
		case FORGOHORDO:
			//�llapot v�ge
			UgyessegiState = FORGOHORDO_RAMPA_FEL;

			//K�vetkez� �llapot eleje
			SpeedSP = 0.5;

			break;
		case FORGOHORDO_RAMPA_FEL:
			//�llapotv�lt�s felt�tele
			if( AngularY < 3 && LineNumFront == 0 )
			{
				//�llapot v�ge
				UgyessegiState = FORGOHORDO_HORDOBAN;
				//K�vetkez� �llapot eleje
				SpeedSP = 0.5;
				osThreadSuspend(SteeringStateSpaceControllerHandle);
				osThreadResume(RollAngleControllerTaskHandle);
			}
			break;
		case FORGOHORDO_HORDOBAN:
			//�llapotv�lt�s felt�tele
			if( LineNumFront == 1 )
			{
				//�llapot v�ge
				UgyessegiState = FORGOHORDO_KI;
				//K�vetkez� �llapot eleje
				SpeedSP = 0.5;
				osThreadSuspend(RollAngleControllerTaskHandle);
				osThreadResume(SteeringStateSpaceControllerHandle);
				Uthossz = 0 ;
			}
			break;
		case FORGOHORDO_KI:
			//�llapotv�lt�s felt�tele
			if( Uthossz > 1 )
			{
				//�llapot v�ge
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

