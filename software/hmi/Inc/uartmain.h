/*
 * uartmain.h
 *
 *  Created on: Dec 3, 2017
 *      Author: smate
 */

#ifndef UARTMAIN_H_
#define UARTMAIN_H_

#include "inttypes.h"

void uart_startReceiving();							//ez h�vd meg initk�nt
void process_uart_msg();						//kapott adatokat dolgozza fel
void uart_send( uint8_t* data , uint8_t size) ;		//add �t az adatbuffert �s a m�ret�t

#endif /* UARTMAIN_H_ */
