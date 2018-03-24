/*
 * uartmain.h
 *
 *  Created on: Dec 3, 2017
 *      Author: smate
 */

#ifndef UARTMAIN_H_
#define UARTMAIN_H_

#include "inttypes.h"

void uart_startReceiving();							//ez hívd meg initként
void process_uart_msg();						//kapott adatokat dolgozza fel
void uart_send( uint8_t* data , uint8_t size) ;		//add át az adatbuffert és a méretét

#endif /* UARTMAIN_H_ */
