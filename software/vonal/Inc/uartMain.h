/*
 * uartmain.h
 *
 *  Created on: 2017. nov. 10.
 *      Author: Vityó
 */

#ifndef UARTMAIN_H_
#define UARTMAIN_H_

#include "inttypes.h"

#ifdef __cplusplus
	#include <infra.h>
#endif

#define DATA_NORMAL 0
#define DATA_DEBUG 1
#define CALIB_CONST_WHITE 2
#define CALIB_CONST_BLACK 3

#ifdef __cplusplus
extern "C"{
#endif

inline void process_uart_msg();

#ifdef __cplusplus
}
#endif




class uartMain {
private:
	char TX_data [300];
	V::infra& infra_ref;
public:
	uartMain(V::infra& infra): infra_ref(infra) {};
	virtual ~uartMain();
	void send( uint8_t mode );
	void startReceiving();
	void notify_readiness();
};

#endif /* UARTMAIN_H_ */
