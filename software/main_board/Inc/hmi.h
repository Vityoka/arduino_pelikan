/*
 * hmi.h
 *
 *  Created on: Nov 7, 2017
 *      Author: smate
 */

#ifndef HMI_H_
#define HMI_H_

#define HMI_BUFF_RX_SIZE 128
#define HMI_BUFF_TX_SIZE 1024

void HMISend(uint16_t size);
void HMICommandParser(void);
void HMIUartRxCallback();
void HMIReceive(void);

#endif /* HMI_H_ */
