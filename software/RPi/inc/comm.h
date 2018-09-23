#ifndef COMM_H
#define COMM_H

#include <iostream>
#include <errno.h>
#include <wiringPiSPI.h>
#include <unistd.h>
#include <string.h> 

int spi_init();
int spi_send_dummy();
int spi_set_SpeedSP( float SpeedSP ); // speed in meter per second
int spi_set_ServoPos( float ServoPos );	//turning angle in radian
int spi_set_Mode_Stop();
int spi_set_Mode_Auto();
int spi_set_Mode_Manual();
int spi_set_Mode_Slave();


#endif /* COMM_H */


