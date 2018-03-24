#include "stm32f0xx_hal.h"

#ifndef MAX7219_H_
#define MAX7219_H_

// spi port
#define MAX7219_SPI_PORT	hspi2
extern SPI_HandleTypeDef MAX7219_SPI_PORT;

// number of chained MAX7219 driven displays
#define NUMBER_OF_DISPLAYS 1

//MAX7219 addresses
#define REG_NOOP   0x00
#define REG_DIGIT0 0x01
#define REG_DIGIT1 0x02
#define REG_DIGIT2 0x03
#define REG_DIGIT3 0x04
#define REG_DIGIT4 0x05
#define REG_DIGIT5 0x06
#define REG_DIGIT6 0x07
#define REG_DIGIT7 0x08
#define REG_DECODEMODE  0x09
#define REG_INTENSITY   0x0A
#define REG_SCANLIMIT   0x0B
#define REG_SHUTDOWN    0x0C
#define REG_DISPLAYTEST 0x0F

///function prototypes:
//Initialize variables and registers
void InitMAX7219(void);

//Update given row of all displays
void updateRow(uint8_t row);

//Update all displays
void updateDisplays(void);

//Send data to device
void setRegister(uint8_t address, uint8_t data);

//Clear buffers and registers
void clear(void);

#endif /* MAX7219_H_ */
