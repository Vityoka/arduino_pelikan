
#ifndef REMOTECONTROL_H_
#define REMOTECONTROL_H_

#include "register_map.h"


#define BIT_kanyar (1 << 0)
#define BIT_motor (1 << 1)
#define BIT_infra (1 << 2)

#define RUN_STOP			1	//kocsi meg�ll
#define RUN_FULL_AUTO				2	//kocsi gyorsul�si m�dban,szab�lyz� futtatja
#define RUN_FULL_MANUAL			3	//kocsi a t�vir�ny�t�val ir�ny�that�
#define RUN_MANUAL_THROTTLE		4	//g�zt a t�vir�ny�t�val adhatunk, de a kanyarod�st a szab�lyz� v�gzi
#define RUN_MANUAL_STEERING	5	//g�zt az alkalmaz�ssal adhatunk, de a kanyarod�st a t�vir�ny�t� v�gzi

void RemoteControllerTask(void const * argument);

extern EventGroupHandle_t Eventgroup_inputCaptures;
//extern TIM_HandleTypeDef htim4;


#endif /* REMOTECONTROL_H_ */
