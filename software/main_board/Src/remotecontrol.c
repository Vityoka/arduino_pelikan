
#include "remotecontrol.h"
#include "controller.h"
#include "ir.h"

uint32_t pulse_kanyar;
uint32_t pulse_motor;
uint32_t period_motor;
uint32_t period_kanyar;


const float KANYAR_RCPWM_MIN = 0.0553;	// 1ms
const float KANYAR_RCPWM_MAX = 0.1157;	// 2ms
const float KANYAR_RCPWM_CENTER = 0.0846;	// 1,5ms
const float MOTOR_RCPWM_MIN = 0.0553;	// 1ms
const float MOTOR_RCPWM_MAX = 0.1264;	// 2ms
const float MOTOR_RCPWM_CENTER = 0.0922;	// 1,5ms

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if( htim->Instance == TIM4)
	{
		pulse_motor = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		period_motor = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		xEventGroupSetBitsFromISR(Eventgroup_inputCaptures , BIT_kanyar , pdFALSE);
	}
	else if(htim->Instance == TIM12)
	{
		pulse_kanyar = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		period_kanyar = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		xEventGroupSetBitsFromISR(Eventgroup_inputCaptures , BIT_motor , pdFALSE);
	}
	else if(htim->Instance == TIM8)
	{
		InfraReceiverInputCaptureCallback();
		xEventGroupSetBitsFromISR(Eventgroup_inputCaptures , BIT_infra , pdFALSE);
	}
}

void RemoteControllerTask(void const * argument)
{
	EventBits_t setBits;
	while(1)
	{
		setBits = xEventGroupWaitBits(
				Eventgroup_inputCaptures ,
				BIT_kanyar | BIT_motor,
				pdTRUE,
				pdFALSE,
				portMAX_DELAY);

		if( ( ( setBits & BIT_kanyar ) != 0 ) )
		{
			if ( Running == RUN_FULL_MANUAL || Running == RUN_MANUAL_STEERING  )
			{
				float ratio_kanyar = (float)pulse_kanyar/(float)period_kanyar;
				if( ratio_kanyar > KANYAR_RCPWM_MAX )
					ratio_kanyar = KANYAR_RCPWM_MAX;
				else if( ratio_kanyar < KANYAR_RCPWM_MIN )
					ratio_kanyar = KANYAR_RCPWM_MIN;
				int ServoSP = (ratio_kanyar-KANYAR_RCPWM_CENTER)/((KANYAR_RCPWM_MAX-KANYAR_RCPWM_CENTER)/1200);
				ServoPos = ServoSP;
			}
		}
		else if( ( ( setBits & BIT_motor ) != 0 ) )
		{
			if ( Running == RUN_FULL_MANUAL || Running == RUN_MANUAL_THROTTLE  )
			{
				float ratio_motor = (float)pulse_motor/(float)period_motor;
				if( ratio_motor > MOTOR_RCPWM_MAX )
					ratio_motor = MOTOR_RCPWM_MAX;
				else if( ratio_motor < MOTOR_RCPWM_MIN )
					ratio_motor = MOTOR_RCPWM_MIN;
				int motorSpeed = (ratio_motor-MOTOR_RCPWM_CENTER)/((MOTOR_RCPWM_MAX-MOTOR_RCPWM_CENTER)/100.0);
				SetMotor(motorSpeed);
			}
			/* DEBUG ONLY */
			/*
			else if( Running == RUN_STOP)
			{
				float ratio_motor = (float)pulse_motor/(float)period_motor;
				if( ratio_motor > MOTOR_RCPWM_MAX )
					ratio_motor = MOTOR_RCPWM_CENTER;
				else if( ratio_motor < MOTOR_RCPWM_MIN )
				ratio_motor = MOTOR_RCPWM_CENTER;
				int motorSpeed = (ratio_motor-MOTOR_RCPWM_CENTER)/((MOTOR_RCPWM_MAX-MOTOR_RCPWM_CENTER)/100.0);
				if(motorSpeed > 50 && CalibRequestState == 0)
					SELFTEST = 0xFF;
			}
			*/
			/*DEBUG ENDS*/

		}
		else	//timeout elapsed
		{

		}


	}
	osThreadTerminate(NULL);
}
