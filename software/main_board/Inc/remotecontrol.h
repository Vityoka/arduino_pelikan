
#ifndef REMOTECONTROL_H_
#define REMOTECONTROL_H_

#include "register_map.h"


#define BIT_kanyar (1 << 0)
#define BIT_motor (1 << 1)
#define BIT_infra (1 << 2)

#define RUN_STOP			1	//kocsi megáll
#define RUN_FULL_AUTO				2	//kocsi gyorsulási módban,szabályzó futtatja
#define RUN_FULL_MANUAL			3	//kocsi a távirányítóval irányítható
#define RUN_MANUAL_THROTTLE		4	//gázt a távirányítóval adhatunk, de a kanyarodást a szabályzó végzi
#define RUN_MANUAL_STEERING	5	//gázt az alkalmazással adhatunk, de a kanyarodást a távirányító végzi

void RemoteControllerTask(void const * argument);

extern EventGroupHandle_t Eventgroup_inputCaptures;
//extern TIM_HandleTypeDef htim4;


#endif /* REMOTECONTROL_H_ */
