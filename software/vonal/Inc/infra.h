/*
 * Infra.h
 *
 *  Created on: 2017. nov. 3.
 *      Author: Vity�
 */

#ifndef INFRA_H_
#define INFRA_H_

#include "inttypes.h"
#include "string.h"

#ifdef __cplusplus
	#include <SPI_infra.h>
#endif

//volatile uint8_t adc_ready_flag;			//a C k�d ezt a h-t includeolva l�tni fogja ezt a flaget, de a classt m�r nem.

#ifdef __cplusplus


	namespace V {

		class infra {
		private:


			int16_t compensated_adc_front[40];
			int16_t compensated_adc_back[24];
			uint16_t calib_consts_white_front[40];
			uint16_t calib_consts_black_front[40];
			uint16_t calib_consts_white_back[24];
			uint16_t calib_consts_black_back[24];
			uint8_t active_sensors [64];

			uint8_t contiguous_regions_front;
			uint8_t contiguous_regions_back;
			uint64_t active_sensors_bitwise;	//bugos,mert nem lehet 32n�l nagyobbakat shiftelni
			uint8_t num_active_sensors_front;
			uint8_t num_active_sensors_back;
			float linepos_front;
			float linepos_back;
			float angle;

			const static int compensated_adc_front_arraybytesize = sizeof(compensated_adc_front) * sizeof(compensated_adc_front[0]);
			const static int compensated_adc_back_arraybytesize = sizeof(compensated_adc_back) * sizeof(compensated_adc_back[0]);
			const static float sensor_dist_diff = 11.6652;				//k�t vonalszenzor k�zti t�vols�g( m�rt�kegys�g: k�t TCRT k�zti t�vols�g )
			const static float middle_of_front_sensorline = 19.5;	//mert 40/2 = 20, vagyis a 20 �s a 21. szenzor k�zt van a k�z�pvonal. De a t�mbindexel�s miatt egyel kisebbet kell annak tekinteni majd.
			const static float middle_of_back_sensorline = 11.5;	//mert 24/2 = 12, vagyis a 12. �s a 13. szenzot k�zt van a k�z�pvonal. De a t�mbindexel�s miatt egyel kisebbet kell annak tekinteni majd.
			const static int threshold = 1500;						//vonal�rz�kel�si k�sz�b
			uint8_t angle_dir;
			enum { left , right } dir ;
			enum { white , black } reflectColor ;

		public:
			uint8_t selectMux;
			//uint16_t new_raw_adc_values[8];
			infra();
			uint64_t getLineLocation();
			uint8_t* getActiveSensors();
			void setValuesForDebug(float linepos_front , float linepos_back, float angle, uint8_t num_front , uint8_t num_back);
			void setCalibForDebug();
			void setMux( uint8_t cycle );
			void calibrate( uint8_t blackWhite );
			void compensate();
			void calculate();
			void count_contigous();
			void startADC();
			void serialize( char* ret , size_t* tx_size , uint8_t mode);
			virtual ~infra();
		};

	} /* namespace V */
#endif


#endif /* INFRA_H_ */
