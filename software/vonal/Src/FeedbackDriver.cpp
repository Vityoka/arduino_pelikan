/*
 * FeedbackDriver.cpp
 *
 *  Created on: 2017. nov. 4.
 *      Author: Vity�
 */

#include <FeedbackDriver.h>
#include "stm32f0xx_hal.h"

extern TIM_HandleTypeDef htim1;

namespace V {

	FeedbackDriver::~FeedbackDriver() {
	}

	void FeedbackDriver::setBrightness( uint8_t dutycycle ) {
		if( dutycycle > 100)			//az OE jel neg�lt, �gy pl a 100-as kit�lt�si t�nyez�h�z 0-t, a 0-hoz 100at kell �ll�tani
			brightness = 0;
		else if( dutycycle < 0 )
			brightness = 100;
		else
			brightness = 100-dutycycle;	//OE jel neg�lt, ez�rt 100-b�l a kit�lt�si t�nyez� kell
		int regvalue = brightness * TIM1_PWM_DUTY_CYCLE_UNIT;
		if ( regvalue > TIM1_PERIOD_REG )
			regvalue = TIM1_PERIOD_REG;

		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, regvalue);
	}

	void FeedbackDriver::driveLeds() {

		uint8_t* active_sensors_ptr = infra_ref.getActiveSensors();
		HAL_GPIO_WritePin(fbLED_SCK_GPIO_Port , fbLED_SCK_Pin , GPIO_PIN_SET);
		HAL_GPIO_WritePin(fbLED_LE_GPIO_Port , fbLED_LE_Pin , GPIO_PIN_RESET); 		//alaphelyzetbe �ll�tom a regiszterbe �tt�lt� jelet

		for(int i = 0; i < 64 ; i++)
		{
			if( active_sensors_ptr[i] == 1 )
				HAL_GPIO_WritePin(fbLED_SDO_GPIO_Port , fbLED_SDO_Pin , GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(fbLED_SDO_GPIO_Port , fbLED_SDO_Pin , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(fbLED_SCK_GPIO_Port , fbLED_SCK_Pin , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(fbLED_SCK_GPIO_Port , fbLED_SCK_Pin , GPIO_PIN_SET);
		}

		HAL_GPIO_WritePin(fbLED_LE_GPIO_Port , fbLED_LE_Pin , GPIO_PIN_SET);		//�tt�lt� jel bekapcsol�sa
		HAL_GPIO_WritePin(fbLED_LE_GPIO_Port , fbLED_LE_Pin , GPIO_PIN_RESET);
	}

	void FeedbackDriver::debug_feedback() {
		while(1)
		{
			uint8_t data [64];
			for(int i = 0; i < 64 ; i++)
			{
				data[i]= 1;
			}
			HAL_GPIO_WritePin(fbLED_SCK_GPIO_Port , fbLED_SCK_Pin , GPIO_PIN_SET);
			HAL_GPIO_WritePin(fbLED_LE_GPIO_Port , fbLED_LE_Pin , GPIO_PIN_RESET); 		//alaphelyzetbe �ll�tom a regiszterbe �tt�lt� jelet

			for(int i = 0; i < 64 ; i++)
			{
				if( data[i] == 1 )
					HAL_GPIO_WritePin(fbLED_SDO_GPIO_Port , fbLED_SDO_Pin , GPIO_PIN_SET);
				else
					HAL_GPIO_WritePin(fbLED_SDO_GPIO_Port , fbLED_SDO_Pin , GPIO_PIN_RESET);
				HAL_GPIO_WritePin(fbLED_SCK_GPIO_Port , fbLED_SCK_Pin , GPIO_PIN_RESET);
				HAL_GPIO_WritePin(fbLED_SCK_GPIO_Port , fbLED_SCK_Pin , GPIO_PIN_SET);
				HAL_Delay(1);
				HAL_GPIO_WritePin(fbLED_LE_GPIO_Port , fbLED_LE_Pin , GPIO_PIN_SET);		//�tt�lt� jel bekapcsol�sa
				HAL_Delay(1);
				HAL_GPIO_WritePin(fbLED_LE_GPIO_Port , fbLED_LE_Pin , GPIO_PIN_RESET);
				HAL_Delay(1);
			}

			for(int i = 0; i < 64 ; i++)
			{
				data[i]= 0;
			}
			HAL_GPIO_WritePin(fbLED_SCK_GPIO_Port , fbLED_SCK_Pin , GPIO_PIN_SET);
			HAL_GPIO_WritePin(fbLED_LE_GPIO_Port , fbLED_LE_Pin , GPIO_PIN_RESET); 		//alaphelyzetbe �ll�tom a regiszterbe �tt�lt� jelet

			for(int i = 0; i < 64 ; i++)
			{
				if( data[i] == 1 )
					HAL_GPIO_WritePin(fbLED_SDO_GPIO_Port , fbLED_SDO_Pin , GPIO_PIN_SET);
				else
					HAL_GPIO_WritePin(fbLED_SDO_GPIO_Port , fbLED_SDO_Pin , GPIO_PIN_RESET);
				HAL_GPIO_WritePin(fbLED_SCK_GPIO_Port , fbLED_SCK_Pin , GPIO_PIN_RESET);
				HAL_GPIO_WritePin(fbLED_SCK_GPIO_Port , fbLED_SCK_Pin , GPIO_PIN_SET);
				HAL_Delay(1);
				HAL_GPIO_WritePin(fbLED_LE_GPIO_Port , fbLED_LE_Pin , GPIO_PIN_SET);		//�tt�lt� jel bekapcsol�sa
				HAL_Delay(1);
				HAL_GPIO_WritePin(fbLED_LE_GPIO_Port , fbLED_LE_Pin , GPIO_PIN_RESET);
				HAL_Delay(1);
			}
		}
	}

	void FeedbackDriver::debug_feedback2() {
		while(1)
		{
			uint8_t data [64];
			for(int i = 0; i < 64 ; i++)
			{
				data[i]= 1;
			}
			HAL_GPIO_WritePin(fbLED_SCK_GPIO_Port , fbLED_SCK_Pin , GPIO_PIN_SET);
			HAL_GPIO_WritePin(fbLED_LE_GPIO_Port , fbLED_LE_Pin , GPIO_PIN_RESET); 		//alaphelyzetbe �ll�tom a regiszterbe �tt�lt� jelet

			for(int i = 0; i < 64 ; i++)
			{
				if( data[i] == 1 )
					HAL_GPIO_WritePin(fbLED_SDO_GPIO_Port , fbLED_SDO_Pin , GPIO_PIN_SET);
				else
					HAL_GPIO_WritePin(fbLED_SDO_GPIO_Port , fbLED_SDO_Pin , GPIO_PIN_RESET);
				HAL_GPIO_WritePin(fbLED_SCK_GPIO_Port , fbLED_SCK_Pin , GPIO_PIN_RESET);
				HAL_GPIO_WritePin(fbLED_SCK_GPIO_Port , fbLED_SCK_Pin , GPIO_PIN_SET);
			}

			HAL_GPIO_WritePin(fbLED_LE_GPIO_Port , fbLED_LE_Pin , GPIO_PIN_SET);		//�tt�lt� jel bekapcsol�sa
			HAL_GPIO_WritePin(fbLED_LE_GPIO_Port , fbLED_LE_Pin , GPIO_PIN_RESET);

			for(int i = 0; i < 64 ; i++)
			{
				data[i]= 0;
			}
			HAL_GPIO_WritePin(fbLED_SCK_GPIO_Port , fbLED_SCK_Pin , GPIO_PIN_SET);
			HAL_GPIO_WritePin(fbLED_LE_GPIO_Port , fbLED_LE_Pin , GPIO_PIN_RESET); 		//alaphelyzetbe �ll�tom a regiszterbe �tt�lt� jelet

			for(int i = 0; i < 64 ; i++)
			{
				if( data[i] == 1 )
					HAL_GPIO_WritePin(fbLED_SDO_GPIO_Port , fbLED_SDO_Pin , GPIO_PIN_SET);
				else
					HAL_GPIO_WritePin(fbLED_SDO_GPIO_Port , fbLED_SDO_Pin , GPIO_PIN_RESET);
				HAL_GPIO_WritePin(fbLED_SCK_GPIO_Port , fbLED_SCK_Pin , GPIO_PIN_RESET);
				HAL_GPIO_WritePin(fbLED_SCK_GPIO_Port , fbLED_SCK_Pin , GPIO_PIN_SET);
			}

			HAL_GPIO_WritePin(fbLED_LE_GPIO_Port , fbLED_LE_Pin , GPIO_PIN_SET);		//�tt�lt� jel bekapcsol�sa
			HAL_GPIO_WritePin(fbLED_LE_GPIO_Port , fbLED_LE_Pin , GPIO_PIN_RESET);
		}
	}

} /* namespace V */
