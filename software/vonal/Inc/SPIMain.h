/*
 * SPIMain.h
 *
 *  Created on: 2017. nov. 4.
 *      Author: Vityó
 */

#ifndef SPIMAIN_H_
#define SPIMAIN_H_

#include <infra.h>

#define TX_BUFSIZE 300
#define MAX_RX_BUF 300
#define INSTRUCTION_LENGTH 5

#ifdef __cplusplus
extern "C"{
#endif

void process_commands();

#ifdef __cplusplus
}
#endif


namespace V {

class SPIMain{
private:
	V::infra& infra_ref;
	char TX_data [TX_BUFSIZE];
public:
	SPIMain(V::infra& infra): infra_ref(infra) {};
	virtual ~SPIMain();
	void send( uint8_t mode );
	void listen();
};

} /* namespace V */



#endif /* SPIMAIN_H_ */
