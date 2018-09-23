/*
 * controller.c
 *
 *  Created on: Dec 10, 2017
 *      Author: smate
 */
#include "controller.h"
#include "math.h"
#include "stdlib.h"
#include "statemachine.h"
#include "inttypes.h"

#define PI  3.14159265358979f

extern SemaphoreHandle_t Semaphore_Uthossz_Achieved;
extern TIM_HandleTypeDef htim2;
extern EventGroupHandle_t Eventgroup_Triggers;


void UthosszTask(const * argument)
{
	volatile uint32_t counter_reg_prev;
	volatile uint32_t counter_reg;
	volatile int32_t delta_reg;
	volatile float velocity = 0;
	const int deltaT = 10;

	while(1)
	{
		//Inkrementális adó beolvasása
		counter_reg = __HAL_TIM_GET_COUNTER(&htim2);
		delta_reg = counter_reg - counter_reg_prev;
		counter_reg_prev = counter_reg;
		velocity = (float)delta_reg / (float)deltaT;
		velocity /= (33312.5 / 1024 )*(7.048/6);
		Speed = -1 * velocity;

		//Uthossz szamítása
		const float magic = 1005.912055034392;	//50ms-es deltaT-hez tartozó konstans
		Uthossz += Speed * deltaT / magic;

		osDelay(deltaT);
	}
	osThreadTerminate(NULL);
}


void RollAngleControllerTask(void const * argument)
{
	float p;
	float e;
	float u;
	float ref = 0;	//alapjel fokban
	float kpHordo = 1;
	while(1)
	{
		//kezdeti értékek
		u = 0;

		//P szabályzó
		e = ref - AngularX;
		p = e * kpHordo;

		//beavatkozó jel elõállítása
		u += p;

		//beavatkozó jel beállítása
		if( Running == RUN_FULL_AUTO || Running == RUN_MANUAL_THROTTLE )
		{
			ServoPos = u * (PI / 180.0f);
		}
		ServoPos = u * (PI / 180.0f);

		osDelay(15);
	}
	osThreadTerminate(NULL);
}


void PIDController(void const * argument)
{
	const uint16_t DeltaT=15;
	float LastError = 0;
	float LastLinePositionFront = 0.0;
	float Error, Pki, Iki, Dki, Out;

	while(1)
	{
		//Hibaszámítás
		if(LineNumBack != 0)
		{
			Error = LinePositionFront;
			LastLinePositionFront = LinePositionFront;
		}
		else
		{
			Error = LastLinePositionFront;
		}
		Out = 0;

		//P szabályozó
		Pki = Error * Kp;
		Out += Pki;

		/*
		//I szabályozó
		Iki= Iki + Ki * error * DeltaT;
		Out+=Iki;
		*/

		//D szabályozó
		Dki= (Kd *(Error - LastError ))/DeltaT;
		Dki *= 1000;
		Out+=Dki;
		Ki = Out;
		//TODO: kimeneti je l beállitása valamivalami(Out)
		if( Running == RUN_FULL_AUTO || Running == RUN_MANUAL_THROTTLE )
		{
			ServoPos = Out;
		}

		//Set last error
		LastError = Error;

		osDelay(DeltaT);
	}
	osThreadTerminate(NULL);
}


void SteeringStateSpaceController(void const * argument){

	const float L = 0.08;	//vonalszenzorok távolsága [m-ben]
	const int e = 0;							//alapjel
	float T;
	float T5percent;
	float u1 , u , p , delta , kp , kdelta;
	float LinePositionFront_prev = LinePositionFront;
	float Vonalszog_prev = LineAngle;

	LinePos_controller = LinePositionFront;
	Vonalszog_controller = LineAngle;

	while(1)
	{
		//vonalpozíció és szög fagyasztása ha lemegyünk a vonalról
		if(LineNumFront == 0)
		{
			LinePos_controller = LinePositionFront_prev;
			Vonalszog_controller = Vonalszog_prev;
		}
		else if(LineNumFront != 0 && LineNumBack == 0)
		{
			Vonalszog_controller = Vonalszog_prev;
			LinePositionFront_prev = LinePos_controller;
			LinePos_controller = LinePositionFront;
		}
		else
		{
			Vonalszog_prev = Vonalszog_controller;
			LinePositionFront_prev = LinePos_controller;
			Vonalszog_controller = LineAngle;
			LinePos_controller = LinePositionFront;
		}

		//vonalszög elõjele lehet hogy ellentétes mint amit szeretnénk:
		//Vonalszog = Vonalszog * (-1);

		D5percent = D5Mul*Speed + D5Add;
		T5percent = D5percent / Speed;
		T = kszi * T5percent / 3;

		//Erõsítések
		kp = -L / (Speed * T * Speed * T);
		kdelta = L / (Speed * T) * (-2 * kszi + L / (Speed * T));

		//súlyozás
		kp *= KpWeight;
		kdelta *= KdeltaWeight;

		//Belsõ változók
		p = kp * LinePos_controller;
		delta = kdelta * Vonalszog_controller;

		u1 = p + delta;

		//Összegzés és kimenet

		u = e - u1;

		if( Running == RUN_FULL_AUTO || Running == RUN_MANUAL_THROTTLE )
		{
			if( abs(Speed) < 0.1 )
				ServoPos = 0;
			else
				ServoPos = u / OutputDivisor;
		}


		osDelay(15);
	}
	osThreadTerminate(NULL);
}

/*
void SpeedControllerTask(void const * argument)
{
	const float K = 0.09014*(6/7.048);
	const float T = 1367.1*(6/7.048);
	const float Ts = 10;
	const float zd = exp(-Ts/T);
	const float Kd_mot = K * (1-zd);

	float Kc;
	float u_prev = 0;
	float u2_prev = 0;
	float u2;
	float u1;
	float u_ki;
	float u;
	float e;
	while(1)
	{
		//Kc számítása
		Kc = 1 / Kd_mot * (1 - exp(-Ts / (float)TclMotor));

		//Alapjel
		e = SpeedSP-Speed;

		//Belsõ változók
		u2 = zd * u2_prev + (1-zd) * u_prev;
		u1 = Kc * e;

		//Összegzés és transzformált szaturáció
		u = u1 + u2;
		if(u > 76)
			u = 76;
		else if (u < -76)
			u = -76;

		//Visszacsatolás
		u_prev = u;
		u2_prev = u2;

		//Inverz statikus karakterisztika
		//törtvonalas karakterisztika közelítése lineárisan
		if(u > 0)
		{
			//u_ki = 30 + u * 0.46;
			u_ki = 50 + u * 0.46;
		}
		else
		{
			//u_ki = -30 + u * 0.46;
			u_ki = -50 + u * 0.46;
		}

		//Nulla közelében motor lekapcsolása
		if(SpeedSP == 0 && (abs(Speed) < 0.2) )
		{
			u_ki = 0;
		}

		//motorra rákapcsoljuk a bravatkozó jelet
		if( Running == RUN_FULL_AUTO || Running == RUN_MANUAL_STEERING || Running == RUN_STOP)
		{
			SetMotor(u_ki);
		}
		osDelay(Ts);
	}
	osThreadTerminate(NULL);
}
*/

void SpeedControllerTask(void const * argument)
{
	const float Ts = 10;
	const float Kp = 0;
	const float Ki = 1;
	float I = 0;

	float u_ki;
	float u;
	float e , P;
	while(1)
	{

		//Alapjel
		e = SpeedSP-Speed;

		P = e * Kp;
		I += e * Ki * Ts;

		u = P + I;

		//Inverz statikus karakterisztika
		//törtvonalas karakterisztika közelítése lineárisan
		if(u > 0)
		{
			//u_ki = 30 + u * 0.46;
			u_ki =  0 + u*0.46;
		}
		else
		{
			//u_ki = -30 + u * 0.46;
			u_ki = 0 + u*0.46;
		}

		//Nulla közelében motor lekapcsolása
		if(SpeedSP == 0 && (abs(Speed) < 0.2) )
		{
			u_ki = 0;
		}

		if( u_ki > 100)
		{
			I -= e * Ki * Ts;
		}
		else if( u_ki < -100 )
		{
			I -= e * Ki * Ts;
		}

		//motorra rákapcsoljuk a bravatkozó jelet
		if( Running == RUN_FULL_AUTO || Running == RUN_MANUAL_STEERING || Running == RUN_STOP)
		{
			SetMotor(u_ki);
		}
		osDelay(Ts);
	}
	osThreadTerminate(NULL);
}

void ServoSetterTask()
{
	const int servo_right_max = 5500;
	const int servo_left_max = 3050;
	const int servo_center = 4500;
	int ServoReg;

	while(1)
	{
		ServoReg = (((servo_right_max - servo_center)/0.349065)*ServoPos) + servo_center;
		if(ServoReg >= servo_right_max)
			ServoReg = servo_right_max;
		else if(ServoReg <= servo_left_max)
			ServoReg = servo_left_max;
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ServoReg);
		osDelay(15);
	}
	osThreadTerminate(NULL);
}

void SetMotor(int motorSpeed)
{
	int SpeedReg;

	if( motorSpeed >= 100 )
	{
		motorSpeed = 99;
	}
	else if( motorSpeed <= -100 )
	{
		motorSpeed = -99;
	}

	//temporary limit of speed, most ugyse használom gyorsulni
	if( motorSpeed > 35)
	{
		motorSpeed = 35;
	}
	else if ( motorSpeed < -35 )
	{
		motorSpeed = -35;
	}

	SpeedReg = (((4500-2250)/100.0)*motorSpeed) + 2250;

	if(Running != RUN_STOP)
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, SpeedReg );
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 4500 - SpeedReg );
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2250);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 4500 - 2250 );
	}
	return;
}

void CarTrackerTask()
{
	int cntr = 0;
	uint8_t n = 0;
	while(1)
	{
		if(DistanceFront > 20)
		{
			MovAvgFrontEnabled = 0;

			SetRollingInfraServo(cntr);

			n++;
			switch(n)
			{
			case 0:
				break;
			case 1:
				cntr = 0;
				break;
			case 2:
				cntr = -80;
				break;
			case 3:
				cntr = -40;
				break;
			case 4:
				cntr = 50;
				break;
			case 5:
				cntr = 100;
				n = 0;
				break;
			}
			osDelay(300);
		}
		else
		{
			MovAvgFrontEnabled = 1;
			n = 0;
			osDelay(50);
		}

	}
	osThreadTerminate(NULL);
}


void SetRollingInfraServo(int angle)
{
	const int servo_right_max = 6000;
	const int servo_left_max = 3500;
	const int servo_center = 4800;
	int ServoReg;

	ServoReg = (((servo_right_max - servo_center)/100)*angle) + servo_center;
	if(ServoReg >= servo_right_max)
		ServoReg = servo_right_max;
	else if(ServoReg <= servo_left_max)
		ServoReg = servo_left_max;
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, ServoReg);
}

