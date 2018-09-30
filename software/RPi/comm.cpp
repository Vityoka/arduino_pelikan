
#include <iostream>
#include <errno.h>
#include <wiringPiSPI.h>
#include <unistd.h>
#include <string.h> 
#include <wiringPi.h>

using namespace std;

// channel is the wiringPi name for the chip select (or chip enable) pin.
// Set this to 0 or 1, depending on how it's connected.
static const int CHANNEL = 1;
unsigned char tx_buffer[100];
float floatbuf = 0;

int spi_init()
{
	int fd, result;
	cout << "Initializing SPI" << endl ;
	// Configure the interface.
	// CHANNEL insicates chip select,
    // 500000 indicates bus speed.
    fd = wiringPiSPISetup(CHANNEL, 500000);	//return with the spi channel's desciptor (fd)
	return fd;
}

int spi_send_dummy()
{
    tx_buffer[0] = 142;
    tx_buffer[1] = 146;
    wiringPiSPIDataRW(CHANNEL, tx_buffer, 10);
	cout << "data sent" << endl;
}

int spi_set_SpeedSP( float SpeedSP )
{

	floatbuf = SpeedSP;
	tx_buffer[0] = 0;
    tx_buffer[1] = 0;
	tx_buffer[2] = 0;
	tx_buffer[3] = 10;
	tx_buffer[4] = 'V';
	memcpy(&tx_buffer[5] , &floatbuf , sizeof(floatbuf));
	int ret = wiringPiSPIDataRW(CHANNEL, tx_buffer, 10);
	delay(10);
	return ret;
}

int spi_set_ServoPos( float ServoPos )
{
	floatbuf = ServoPos;
	tx_buffer[0] = 0;
    tx_buffer[1] = 0;
	tx_buffer[2] = 0;
	tx_buffer[3] = 10;
	tx_buffer[4] = 's';
	memcpy(&tx_buffer[5] , &floatbuf , sizeof(floatbuf));
	int ret = wiringPiSPIDataRW(CHANNEL, tx_buffer, 10);
	delay(10);
	return ret;
}

int spi_set_Mode_Stop()
{
	tx_buffer[0] = 0;
    tx_buffer[1] = 0;
	tx_buffer[2] = 0;
	tx_buffer[3] = 10;
	tx_buffer[4] = 'S';
	int ret = wiringPiSPIDataRW(CHANNEL, tx_buffer, 10);
	delay(10);
	return ret;
}

int spi_set_Mode_Auto()
{
	tx_buffer[0] = 0;
    tx_buffer[1] = 0;
	tx_buffer[2] = 0;
	tx_buffer[3] = 10;
	tx_buffer[4] = 'G';
	tx_buffer[5] = 2;

	int ret = wiringPiSPIDataRW(CHANNEL, tx_buffer, 10);
	delay(10);
	return ret;
}

int spi_set_Mode_Manual()
{
	tx_buffer[0] = 0;
    tx_buffer[1] = 0;
	tx_buffer[2] = 0;
	tx_buffer[3] = 10;
	tx_buffer[4] = 'G';
	tx_buffer[5] = 3;

	int ret = wiringPiSPIDataRW(CHANNEL, tx_buffer, 10);
	delay(10);
	return ret;
}



int spi_set_Mode_Slave()
{
	tx_buffer[0] = 0;
        tx_buffer[1] = 0;
	tx_buffer[2] = 0;
	tx_buffer[3] = 10;
	tx_buffer[4] = 'y';
	int ret = wiringPiSPIDataRW(CHANNEL, tx_buffer, 10);
	delay(10);
	return ret;
}
/*
int main()
{
	spi_init();
	spi_set_Mode_Slave();
	spi_set_Mode_Auto();
	while(1)
	{

		spi_set_SpeedSP(0.15);
		sleep(2);
		spi_set_SpeedSP(0.3);
		sleep(2);
		spi_set_ServoPos(0.2);
		spi_set_SpeedSP(0);
		sleep(2);
		spi_set_SpeedSP(0.1);
		sleep(2);
		spi_set_SpeedSP(0);
		sleep(2);
		spi_set_ServoPos(-0.2);
		spi_set_SpeedSP(-0.1);
		sleep(2);
		spi_set_SpeedSP(0);
	}
}
*/

