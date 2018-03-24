/*
 * mpu6050.h
 *
 *  Created on: 2018. jan. 28.
 *      Author: Vityó
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "inttypes.h"



#define IMU_ADDR				0x68	//0x68

// -- DEFINES OF REGISTER ADDRESSES AND DEFAULT CONFIG VALUES -- //
#define PWR_MGMT_1				107	//0x6B
#define PWR_MGMT_1_TEMP_DIS		8	//to disable temperature sensor
#define PWR_MGMT_1_DEFAULT		0	//indításkor ez kell hogy beinduljon  a szenzor , wakeup
#define PWR_MGMT_1_DEVICE_RESET	0x80//reset

#define SMPRT_DIV				25	//0x19
/*This register specifies the divider from the gyroscope output rate used to generate the Sample Rate for the MPU-60X0.
The sensor register output, FIFO output, and DMP sampling are all based on the Sample Rate.*/
#define SMPRT_DIV_VAL			8	//leosztunk 8-al

#define CONFIG					26	//0x1A
#define CONFIG_VAL				0

#define GYRO_CONFIG				27	//0x1B
#define GYRO_FS250				0
#define GYRO_FS500				0x08

#define ACCEL_CONFIG			28	//0x1C
#define ACCEL_FS2g				0x00
#define ACCEL_FS4g				0x08
#define ACCEL_FS8g				0x10
#define ACCEL_FS16g				0x18

#define FIFO_EN					35	//0x23
#define FIFO_EN_GYRO_ACCEL_EN	0x78	//120 //01111000b

#define INT_PIN_CFG 			55 //0x37 , IT signal config reg
#define INT_PIN_CFG_INT_RD_CLR	0x10	//törlõdik az IT status minden olvasási mûveletkor, nem csak az INT_STATUS olvasásakor

#define INT_ENABLE					56	//0x38
#define INT_ENABLE_DATA_RDY_EN		0x01	//if DataReady
#define INT_ENABLE_FIFO_OFLOW_EN	0x10	//if DataReady

#define USER_CTRL					106	//0x6A
#define USER_CTRL_FIFO_RESET		0x04	//resets fifo
#define USER_CTRL_FIFO_EN			0x40	//enables fifo
#define USER_CTRL_SIG_COND_RESET	0x01	//törli a szenzor regiszter értékeket

#define SIGNAL_PATH_RESET			0x68	//reseteli a gyrot, accelerot



#define ACCEL_XOUT_H	59	//0x3B
#define ACCEL_XOUT_L	60	//0x3C
#define ACCEL_YOUT_H	61	//0x3D
#define ACCEL_YOUT_L	62	//0x3E
#define ACCEL_ZOUT_H	63	//0x3F
#define ACCEL_ZOUT_L	64	//0x40

#define GYRO_XOUT_H		67	//0x43
#define GYRO_XOUT_L		68	//0x44
#define GYRO_YOUT_H		69	//0x45
#define GYRO_YOUT_L		70	//0x46
#define GYRO_ZOUT_H		71	//0x47
#define GYRO_ZOUT_L		72	//0x48

#define WHO_AM_I		117	//0x75
#define MEMS_ID			0x68

void getSensorValuesSlow();
uint8_t i2c_write_byte(uint8_t addr, uint8_t data);
uint8_t i2c_read_byte(uint8_t addr);
void i2c_read_burst(uint8_t addr , uint8_t* destination, uint8_t size);
void HAL_I2C_ClearBusyFlagErrata_2_14_7(I2C_HandleTypeDef *hi2c) ;

#endif /* MPU6050_H_ */
