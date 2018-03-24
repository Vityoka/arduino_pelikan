/*
 * ir.c
 *
 *  Created on: Nov 7, 2017
 *      Author: smate
 */

#include "register_map.h"
#include "ir.h"

#define BIT_infra (1 << 2)

enum IT_state {POL_LOW , POL_HIGH};
enum pulsetype {LOW_SHORT , LOW_LONG , HIGH_SHORT , HIGH_LONG , PULSE_ERROR};
enum RC5_state {RC5_RESTART , RC5_MID_1 , RC5_START_1 , RC5_MID_0 , RC5_START_0};

enum IT_state IT_state = POL_HIGH;
enum pulsetype pulsetype;
enum RC5_state RC5_state = RC5_RESTART;

uint8_t RC5_bitstream [14];
uint8_t data_invalid = 0;
uint8_t addr_invalid = 0;
uint16_t pulse_T1 = 0;
uint16_t pulse_T2 = 0;
uint16_t pulselength = 0;

uint16_t tempTimes[100];
uint8_t indexx = 0;
int n = 0;

void InfraReceiveTask(void const * argument){

	for( int i = 0; i< 50; i++)
	{
		tempTimes[i] = 0;
	}
	EventBits_t setBits;
	while(1)
	{
		setBits = xEventGroupWaitBits(
				Eventgroup_inputCaptures ,
				BIT_infra,
				pdTRUE,
				pdFALSE,
				portMAX_DELAY);

		if( ( setBits & BIT_infra ) != 0 )
		{
			//MX_TIM8_Init();
			//TIM8_ReInit_IC();
			//CalcPulseType();
			//decode_RC5();
		}
		else	//timeout elapsed
		{

		}
	}
	osThreadTerminate(NULL);
}

void evaluateInfraBitsream()
{
	for(int i = 3 ; i < 8 ; i++)	//check if address is valid
	{
		if( RC5_bitstream[i] != 0x0D )
		{
			addr_invalid = 1;
			break;
		}
	}
	if( addr_invalid == 0 )
	{
		for(int i = 9 ; i < 14 ; i++) //check if data is valid
		{
			if( RC5_bitstream[i] != RC5_bitstream[i-1] || RC5_bitstream[i] > 6 || RC5_bitstream[i] < 1 )
			{
				data_invalid = 1;
				break;
			}
		}
		if( data_invalid == 0)
		{
			KorforgalomData = RC5_bitstream[10];
		}
		else
		{
			data_invalid = 0;
		}
	}
	else
	{
		addr_invalid = 0;
	}

}


inline void decode_RC5(){

	uint8_t bitindex = 0;

	switch(RC5_state)
	{
	case RC5_RESTART:
		FlushIRData();
		bitindex = 0;
		if(pulsetype != PULSE_ERROR )
		{
			RC5_state = RC5_MID_1;
			RC5_bitstream[bitindex] = 1;
		}
		break;
	case RC5_MID_1:
		if(pulsetype == HIGH_SHORT )
		{
			RC5_state = RC5_START_1;
		}
		else if( pulsetype == HIGH_LONG )
		{
			RC5_state = RC5_MID_0;
			RC5_bitstream[bitindex] = 0;
		}
		else
		{
			RC5_state = RC5_RESTART;
		}
		break;
	case RC5_START_1:
		if(pulsetype == LOW_SHORT )
		{
			RC5_state = RC5_MID_1;
			RC5_bitstream[bitindex] = 1;
		}
		else
		{
			RC5_state = RC5_RESTART;
		}
		break;
	case RC5_MID_0:
		if(pulsetype == LOW_LONG )
		{
			RC5_state = RC5_MID_1;
			RC5_bitstream[bitindex] = 1;
		}
		else if( pulsetype == LOW_SHORT )
		{
			RC5_state = RC5_START_0;
		}
		else
		{
			RC5_state = RC5_RESTART;
		}
		break;
	case RC5_START_0:
		if(pulsetype == HIGH_SHORT )
		{
			RC5_state = RC5_MID_0;
			RC5_bitstream[bitindex] = 0;
		}
		else
		{
			RC5_state = RC5_RESTART;
		}
		break;
	}
	bitindex++;
	if( bitindex >= 14 )
	{
		evaluateInfraBitsream();
		bitindex = 0;
		for(int i = 0 ; i < 14 ; i++)
		{
			IR_Data[i] = RC5_bitstream[i];
		}
	}
}

inline void CalcPulseType(){

	const int PULSELENGTH_LONG_MAX = 1000;
	const int PULSELENGTH_LONG_MIN = 1000;
	const int PULSELENGTH_SHORT_MAX = 1000;
	const int PULSELENGTH_SHORT_MIN = 1000;

	if(pulselength < PULSELENGTH_LONG_MAX && pulselength > PULSELENGTH_LONG_MIN)
	{
		if( IT_state == POL_HIGH )
			pulsetype = HIGH_LONG;
		else
			pulsetype = LOW_LONG;
	}
	else if(pulselength < PULSELENGTH_SHORT_MAX && pulselength > PULSELENGTH_SHORT_MIN)
	{
		if( IT_state == POL_HIGH )
			pulsetype = HIGH_SHORT;
		else
			pulsetype = LOW_SHORT;
	}
	else
		pulsetype = PULSE_ERROR;
}

inline void InfraReceiverInputCaptureCallback(){
	if( IT_state == POL_HIGH )	//H-ban voltunk, vagyis lefutó él érkezett
	{
		pulse_T1 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_3);

		//read pinvalue to determine if it was rising or falling edge
		if( HAL_GPIO_ReadPin(IR_Data_GPIO_Port , IR_Data_Pin) == 1)
		{
			IT_state = POL_HIGH;
		}
		else
		{
			IT_state = POL_LOW;
		}
		pulselength = pulse_T1 - pulse_T2;
		tempTimes[indexx] = pulselength;
		indexx++;
	}
	else					//L-ben voltunk, vagyis felfutó él érkezett
	{
		pulse_T2 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_3);
		//read pinvalue to determine if it was rising or falling edge
		if( HAL_GPIO_ReadPin(IR_Data_GPIO_Port , IR_Data_Pin) == 1)
		{
			IT_state = POL_HIGH;
		}
		else
		{
			IT_state = POL_LOW;
		}
		pulselength = pulse_T2 - pulse_T1;
		tempTimes[indexx] = pulselength;
		indexx++;
		//pulse_T1 = 0;
		//pulse_T2 = 0;
		//__HAL_TIM_SetCounter(&htim8, 0);
	}
/*
	n++;
	if( n >= 100)
		asm("bkpt 255");
*/
	CalcPulseType();
}


inline void FlushIRData(){
	for( int i = 0;  i < 14 ; i++)
	{
		IR_Data[i] = 0;
	}
}




