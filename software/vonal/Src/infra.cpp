/*
 * Infra.cpp
 *
 *  Created on: 2017. nov. 3.
 *      Author: Vity�
 */
#include "main.h"
#include "uartMain.h"
#include <infra.h>
#include "stm32f0xx_hal.h"
#include "math.h"
#include "string.h"

using namespace V;

extern ADC_HandleTypeDef hadc;
extern volatile uint8_t adc_ready_flag;
uint16_t new_raw_adc_values[8];
uint16_t debug_adc_ch3;

infra::infra() {
	//TODO kinull�zni midnen t�mb�t
	for(int i = 0 ; i < 8 ; i++)
	{
		new_raw_adc_values[i] = 0;
	}
	for( int i = 0 ; i< 40 ; i++ )
	{
		calib_consts_white_front[i] = 0;
	}
	for( int i = 0 ; i < 24 ; i++ )
	{
		calib_consts_white_back[i] = 0;
	}
	for( int i = 0 ; i< 40 ; i++ )
	{
		calib_consts_black_front[i] = 0;
	}
	for( int i = 0 ; i < 24 ; i++ )
	{
		calib_consts_black_back[i] = 0;
	}
	for( int i = 0 ; i < 64 ; i++ )
	{
		active_sensors[i] = 0;
	}
	linepos_back = 0;
	linepos_front = 0;
	active_sensors_bitwise = 0;
	num_active_sensors_front = 0;
	num_active_sensors_back = 0;
	HAL_GPIO_WritePin(EN_sensors_GPIO_Port , EN_sensors_Pin , GPIO_PIN_RESET);	//MUX-ok enged�lyez�se, 0-ra akt�v
}

infra::~infra() {
}

uint64_t infra::getLineLocation(){
	return active_sensors_bitwise;
}
uint8_t* infra::getActiveSensors() {
	return active_sensors;
}


void infra::serialize(char* ret , size_t* tx_size , uint8_t mode) {


	char buf [50];
	switch(mode)
	{
		case DATA_NORMAL:
			memcpy(ret , &linepos_front , sizeof(linepos_front));
			*tx_size += sizeof(linepos_front);
			memcpy(ret + *tx_size , &linepos_back , sizeof(linepos_back));
			*tx_size += sizeof(linepos_back);
			memcpy(ret + *tx_size, &angle , sizeof(angle));
			*tx_size += sizeof(angle);
			memcpy(ret + *tx_size, &contiguous_regions_front , sizeof(contiguous_regions_front));
			*tx_size += sizeof(contiguous_regions_front);
			memcpy(ret + *tx_size, &contiguous_regions_back , sizeof(contiguous_regions_back));
			*tx_size += sizeof(contiguous_regions_back);
			memcpy(ret + *tx_size, &num_active_sensors_front , sizeof(num_active_sensors_front));
			*tx_size += sizeof(num_active_sensors_front);
			memcpy(ret + *tx_size, &num_active_sensors_back , sizeof(num_active_sensors_back));
			*tx_size += sizeof(num_active_sensors_back);
			break;
		case DATA_DEBUG:
			memcpy(ret , &linepos_front , sizeof(linepos_front));
			*tx_size += sizeof(linepos_front);
			memcpy(ret + *tx_size , &linepos_back , sizeof(linepos_back));
			*tx_size += sizeof(linepos_back);
			memcpy(ret + *tx_size, &angle , sizeof(angle));
			*tx_size += sizeof(angle);
			memcpy(ret + *tx_size, &contiguous_regions_front , sizeof(contiguous_regions_front));
			*tx_size += sizeof(contiguous_regions_front);
			memcpy(ret + *tx_size, &contiguous_regions_back , sizeof(contiguous_regions_back));
			*tx_size += sizeof(contiguous_regions_back);
			memcpy(ret + *tx_size, &num_active_sensors_front , sizeof(num_active_sensors_front));
			*tx_size += sizeof(num_active_sensors_front);
			memcpy(ret + *tx_size, &num_active_sensors_back , sizeof(num_active_sensors_back));
			*tx_size += sizeof(num_active_sensors_back);
			memcpy( ret + *tx_size , compensated_adc_front , compensated_adc_front_arraybytesize);
			*tx_size += compensated_adc_front_arraybytesize;
			memcpy( ret + *tx_size , compensated_adc_back , compensated_adc_back_arraybytesize);
			*tx_size += compensated_adc_back_arraybytesize;
			break;
		case CALIB_CONST_BLACK:
			memcpy( ret , calib_consts_white_front , 40*2);
			memcpy( ret + 40*2 , calib_consts_white_back , 2*24);
			*tx_size = 40*2 + 2*24;
			break;
		case CALIB_CONST_WHITE:
			memcpy( ret , calib_consts_black_front , 40*2);
			memcpy( ret + 40*2 , calib_consts_black_back , 2*24);
			*tx_size = 40*2 + 2*24;
			break;
	}
}


void infra::calibrate( uint8_t blackWhite ) {

	for( int i = 0 ; i < 3 ; i++ )	//3db 8-as rekeszen l�pked�nk v�gig
	{
		if( blackWhite == CALIB_CONST_WHITE )
			calib_consts_white_back[ i*8 + selectMux ] = new_raw_adc_values[i];
		else
			calib_consts_black_back[ i*8 + selectMux ] = new_raw_adc_values[i];
	}

	int i,j;
	for( i = 0 , j = 3 ;  i < 5 ; i++ , j++)	//5db 8-as rekeszen l�pked�nk v�gig
	{
		if( blackWhite == CALIB_CONST_WHITE )
			calib_consts_white_front[ i*8 + selectMux ] = new_raw_adc_values[j];
		else
			calib_consts_black_front[ i*8 + selectMux ] = new_raw_adc_values[j];
	}

}

void infra::setMux( uint8_t cycle ) {				//MUX-ok (csatorn�k) be�ll�t�sa, hogy j� �rt�ket mintav�telezz�nk.
	if( (cycle <= 7) && (cycle >= 0) )
	{
		selectMux = cycle;
		switch(selectMux)
		{
		case 0:
			HAL_GPIO_WritePin(Select0_GPIO_Port , Select0_Pin , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Select1_GPIO_Port , Select1_Pin , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Select2_GPIO_Port , Select2_Pin , GPIO_PIN_RESET);
			break;
		case 1:
			HAL_GPIO_WritePin(Select0_GPIO_Port , Select0_Pin , GPIO_PIN_SET);
			HAL_GPIO_WritePin(Select1_GPIO_Port , Select1_Pin , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Select2_GPIO_Port , Select2_Pin , GPIO_PIN_RESET);
			break;
		case 2:
			HAL_GPIO_WritePin(Select0_GPIO_Port , Select0_Pin , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Select1_GPIO_Port , Select1_Pin , GPIO_PIN_SET);
			HAL_GPIO_WritePin(Select2_GPIO_Port , Select2_Pin , GPIO_PIN_RESET);
			break;
		case 3:
			HAL_GPIO_WritePin(Select0_GPIO_Port , Select0_Pin , GPIO_PIN_SET);
			HAL_GPIO_WritePin(Select1_GPIO_Port , Select1_Pin , GPIO_PIN_SET);
			HAL_GPIO_WritePin(Select2_GPIO_Port , Select2_Pin , GPIO_PIN_RESET);
			break;
		case 4:
			HAL_GPIO_WritePin(Select0_GPIO_Port , Select0_Pin , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Select1_GPIO_Port , Select1_Pin , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Select2_GPIO_Port , Select2_Pin , GPIO_PIN_SET);
			break;
		case 5:
			HAL_GPIO_WritePin(Select0_GPIO_Port , Select0_Pin , GPIO_PIN_SET);
			HAL_GPIO_WritePin(Select1_GPIO_Port , Select1_Pin , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Select2_GPIO_Port , Select2_Pin , GPIO_PIN_SET);
			break;
		case 6:
			HAL_GPIO_WritePin(Select0_GPIO_Port , Select0_Pin , GPIO_PIN_RESET);
			HAL_GPIO_WritePin(Select1_GPIO_Port , Select1_Pin , GPIO_PIN_SET);
			HAL_GPIO_WritePin(Select2_GPIO_Port , Select2_Pin , GPIO_PIN_SET);
			break;
		case 7:
			HAL_GPIO_WritePin(Select0_GPIO_Port , Select0_Pin , GPIO_PIN_SET);
			HAL_GPIO_WritePin(Select1_GPIO_Port , Select1_Pin , GPIO_PIN_SET);
			HAL_GPIO_WritePin(Select2_GPIO_Port , Select2_Pin , GPIO_PIN_SET);
			break;
		}
	}

}


void infra::startADC() {			//csatorna be�ll�t�sa ut�n szoftveresen elind�tjuk az ADC konverzi�t
	adc_ready_flag = 0;
	HAL_ADC_Start_DMA(&hadc , (uint32_t*)new_raw_adc_values , 8 );		//8 adatot kell bem�solni, a DMA m�r be van konfigolva �gy hogy f�lsazvank�nt(16bit) helyezze le a k�vetkez� adatot, mert 16 bites �rt�kek vannak a t�mbben amibe m�sol.
}

/*
void infra::debugADC_CH3() {
	adc_ready_flag = 0;
	HAL_ADC_Start(&hadc1);

}
*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	adc_ready_flag = 1;
}

void infra::compensate() {			//kalibr�ci�s konstansokkal val� kompenz�l�s, �s az �rt�kek sz�tsz�r�sa a megfelel� t�mbbe.

	//jelenleg csak a black-el kompenz�l

	int16_t newval = 0;
	for( int i = 0 ; i < 3 ; i++ )	//3db 8-as rekeszen l�pked�nk v�gig
	{
		newval = new_raw_adc_values[i] - calib_consts_black_back[i*8 + selectMux];	//kompenz�l�s a kalibr�ci�s konstansokkal
		compensated_adc_back[ i*8 + selectMux ] = newval;
	}

	int i,j;
	for( i = 0 , j = 3 ;  i < 5 ; i++ , j++)	//5db 8-as rekeszen l�pked�nk v�gig
	{
		newval = new_raw_adc_values[j] - calib_consts_black_front[i*8 + selectMux];	//kompenz�l�s a kalibr�ci�s konstansokkal
		compensated_adc_front[ i*8 + selectMux ] = newval;
	}


}

void infra::calculate() {		//s�lyozott �tlagol�s

	int32_t sum = 0;
	int32_t sum_pos = 0;
	active_sensors_bitwise = 0;
	num_active_sensors_front = 0;
	num_active_sensors_back = 0;
	for( int i = 0 ; i < 64 ; i++ )
	{
		active_sensors[i] = 0;
	}

	//els� szenzorsor sz�m�t�sa: s�lyozzott �tlag �s k�sz�b�z�s
	for(int i = 0; i < 40 ; i++)
	{
		if(compensated_adc_front[i] > threshold)		//k�sz�b�z�
		{
			num_active_sensors_front ++;
			//active_sensors_bitwise |= ( 1 << i );
			active_sensors[40-i-1] = 1;
		}
		sum += compensated_adc_front[i];			//s�lyozott �tlagol�
		sum_pos += i * compensated_adc_front[i];
	}
	linepos_front = ((float)sum_pos/(float)sum) - middle_of_front_sensorline;	//-1 a t�mbindexel�s miatt kell

	//h�ts� szenzorsor sz�m�t�sa: s�lyozzott �tlag �s k�sz�b�z�s
	sum = 0;
	sum_pos = 0;
	for(int i = 0; i < 24 ; i++)
	{
		if(compensated_adc_back[i] > threshold)		//k�sz�b�z�
		{
			num_active_sensors_back++;
			//active_sensors_bitwise |= ( 1 << (i+40));
			active_sensors[40 + 24 - i - 1] = 1;
		}
		sum += compensated_adc_back[i];				//s�lyozott �tlagol�
		sum_pos += i * compensated_adc_back[i];
	}
	linepos_back = ((float)sum_pos/(float)sum) - middle_of_back_sensorline;	//-1 a t�mbindexel�s miatt kell

	//orient�ci� �s ir�ny sz�m�t�sa
	angle = atan((linepos_front-linepos_back)/sensor_dist_diff);				//ezt gyors�tani k�ne

	if( linepos_front-linepos_back > 0 )
		angle_dir = left;
	else
		angle_dir = right;


	count_contigous();
}

void infra::count_contigous() {
	uint8_t cntr = 0;			//szamolja az egymas melletti infrak szamat
	contiguous_regions_back = 0;
	contiguous_regions_front = 0;
	for(int i = 0 ; i < 24; i++)
	{
		if( compensated_adc_back[i] > threshold )	//ha aktiv az adott infra
		{
			cntr++;
		}
		else	//ha inaktiv, akkor vegeszakadt egy egybefuggo regionak, vagyis akkor noveljuk egyel az egybefuggo aktiv regiok szamat
		{
			if( cntr != 0 )
			{
				contiguous_regions_back++;
				cntr = 0;
			}
			else
			{
				cntr = 0;
			}
		}
	}
	if(cntr != 0)	//ha regio van a vonalszenzor szelen nem lenne jo az algoritmus mert csak 24ig meg a ciklus, ezert megint le kell tesztelni
	{
		contiguous_regions_back++;
		cntr = 0;
	}

	for(int i = 0 ; i < 40; i++)
	{
		if( compensated_adc_front[i] > threshold )
		{
			cntr++;
		}
		else
		{
			if( cntr != 0 )
			{
				contiguous_regions_front++;
				cntr = 0;
			}
			else
			{
				cntr = 0;
			}
		}
	}
	if(cntr != 0)	//ha regio van a vonalszenzor szelen nem lenne jo az algoritmus mert csak 24ig meg a ciklus, ezert megint le kell tesztelni
	{
		contiguous_regions_front++;
	}
}

void infra::setValuesForDebug(float linepos_front, float linepos_back, float angle, uint8_t num_front, uint8_t num_back) {
	this->linepos_front = linepos_front;
	this->linepos_back = linepos_back;
	this->angle = angle;
	this->num_active_sensors_back = num_back;
	this->num_active_sensors_front = num_front;
}

void infra::setCalibForDebug() {
	for( int i = 0 ; i< 40 ; i++ )
	{
		calib_consts_white_front[i] = 11;
	}
	for( int i = 0 ; i < 24 ; i++ )
	{
		calib_consts_white_back[i] = 22;
	}
	for( int i = 0 ; i< 40 ; i++ )
	{
		calib_consts_black_front[i] = 33;
	}
	for( int i = 0 ; i < 24 ; i++ )
	{
		calib_consts_black_back[i] = 44;
	}
	for( int i = 0 ; i < 24 ; i++ )
	{
		compensated_adc_back[i] = 55;
	}
	for( int i = 0 ; i < 24 ; i++ )
	{
		compensated_adc_front[i] = 66;
	}

}




//C wrapper
/*
extern "C" void getRawArrays( infra* ptr ){
	return ptr->setRawValues();
}
*/
//C wrapper 2

extern "C" void randomfugveny( void* myClass ){
	//infra* infra = static_cast<*myClass>
	infra* temp = (infra*)myClass;
	temp->calculate();
}


