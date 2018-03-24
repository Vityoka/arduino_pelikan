/*
 * SPIinfra.h
 *
 *  Created on: 2017. nov. 4.
 *      Author: Vityó
 */

#ifndef SPI_INFRA_H_
#define SPI_INFRA_H_

#include "inttypes.h"



class SPI_infra {
private:
	uint8_t Tx_buffer [8];
public:
	SPI_infra();
	virtual ~SPI_infra();
	void send(uint8_t cycle);
	void debug_infra();
	void debug_infra2();
	void debug_infra3();
	void debug_infra4();
	void debug_infra5();
};

#endif /* SPI_INFRA_H_ */
