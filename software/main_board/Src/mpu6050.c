/*
 * mpu6050.c
 *
 *  Created on: 2018. jan. 28.
 *      Author: Vityó
 */

#include "mpu6050.h"
#include "register_map.h"
#include "math.h"

#define PI 3.14159265359

extern I2C_HandleTypeDef hi2c1;
extern SemaphoreHandle_t Semaphore_MEMS_ready;

uint8_t MemsTxBuf [5];

int16_t AccX;
int16_t AccY;
int16_t AccZ;
int16_t GyroX;
int16_t GyroY;
int16_t GyroZ;

float GyroXCalib = 0;
float GyroYCalib = 0;
float GyroZCalib = 0;

const float GYRO_SENS_FS250 = 131;
const float GYRO_SENS_FS500 = 65.5;
const float GYRO_SENS_FS1000 = 32.8;
const float GYRO_SENS_FS2000 = 16.4;

uint8_t AccDataRaw [6];
uint8_t GyroDataRaw [6];

const float deltaTsec = 0.01;

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c->Instance == I2C1)
	{
		int fasza = 1;
	}
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c->Instance == I2C1)
	{
		xSemaphoreGiveFromISR( Semaphore_MEMS_ready , NULL );
	}
}

void IMU_Init(void)
{
	//HAL_I2C_ClearBusyFlagErrata_2_14_7(&hi2c1);	//bugos az stmen az i2c ezért kell egy függvény ami kitörli a busy flaget. csak az a helyzet hogy ez a függvény se segít xd

	if( i2c_read_byte(WHO_AM_I ) != MEMS_ID)
	{
		//asm("bkpt");
	}

	//reset
	if( i2c_write_byte(PWR_MGMT_1, PWR_MGMT_1_DEFAULT) != HAL_OK )		//kopmplett eszköz reset, indításhoz kell.
	{
		//asm("bkpt");
	}
	//i2c_write_byte(USER_CTRL, USER_CTRL_SIG_COND_RESET);	//regiszter reset
	//i2c_write(USER_CTRL, USER_CTRL_FIFO_RESET);		//fifo reset


	//i2c_write_byte(PWR_MGMT_1, PWR_MGMT_1_DEFAULT);		//normál mûködéshez, wakeup
	//i2c_write(INT_PIN_CFG, INT_PIN_CFG_INT_RD_CLR); //törlõdik az IT status minden olvasási mûveletkor, nem csak az INT_STATUS olvasásakor
	//i2c_write_byte(SMPRT_DIV, SMPRT_DIV_VAL)	;			//leosztjuk a gyro sample ratejét ami 8khZ, az acc-é 1 kHZ. Most nem osztjuk  le, 0 az érték, szal nem is kéne ez
	//i2c_write(INT_ENABLE, INT_ENABLE_DATA_RDY_EN);	//Data_ready IT, amikor van új data IT-t dob az IMU
	//i2c_write(CONFIG, CONFIG_VAL);

	if( i2c_write_byte(GYRO_CONFIG, GYRO_FS500) != HAL_OK )
	{
		//asm("bkpt");
	}
	if( i2c_write_byte(ACCEL_CONFIG, ACCEL_FS8g) != HAL_OK )
	{
		//asm("bkpt");
	}

	//i2c_write(FIFO_EN, FIFO_EN_GYRO_ACCEL_EN);	//engedélyezi a fifoba az accelero és a gyro írását
	//i2c_write_byte(FIFO_EN, 0);							//fifo disable

	xSemaphoreGive( Semaphore_MEMS_ready );
}

void IMU_Calibrate()
{
	long GyroXAccu = 0;
	long GyroYAccu = 0;
	long GyroZAccu = 0;

	for (int cal_int = 0; cal_int < 100 ; cal_int ++){                  //Read the raw acc and gyro data from the MPU-6050 for 1000 times
		getSensorValuesSlow();
		GyroXAccu += GyroX;                                              //Add the gyro x offset to the GyroXCalib variable
		GyroYAccu += GyroY;                                              //Add the gyro y offset to the GyroYCalib variable
		GyroZAccu += GyroZ;                                              //Add the gyro z offset to the GyroZCalib variable
		osDelay(deltaTsec*1000);
	}

	// divide by 1000 to get avarage offset
	GyroXCalib = GyroXAccu / 100.0;
	GyroYCalib = GyroYAccu / 100.0;
	GyroZCalib = GyroZAccu / 100.0;

}

uint8_t i2c_write_byte(uint8_t addr, uint8_t data)
{
	uint8_t pData[2];
	pData[0] = addr;
	pData[1] = data;
	return HAL_I2C_Master_Transmit(&hi2c1, IMU_ADDR<<1, pData, 2 , 50);
}


uint8_t i2c_read_byte(uint8_t addr)
{
	uint8_t pData[2];
	pData[0] = addr;
	pData[1] = 0;

	int retval = HAL_I2C_Master_Transmit(&hi2c1, IMU_ADDR<<1, pData, 1 , 50);
	int retval2 = HAL_I2C_Master_Receive(&hi2c1, IMU_ADDR<<1, pData, 1 , 50);
	if( retval != HAL_OK)
	{
		//asm("bkpt");
	}
	if( retval2 != HAL_OK)
	{
		//asm("bkpt");
	}

	return pData[0];
}

void i2c_read_burst(uint8_t addr , uint8_t* destination, uint8_t size)	//nem mûködik
{
	if ( xSemaphoreTake( Semaphore_MEMS_ready, 500)  != pdTRUE)
	{
		//asm("bkpt");
	}
	else
	{
		MemsTxBuf[0] = addr;
		MemsTxBuf[1] = 0;
		int retval = HAL_I2C_Master_Transmit(&hi2c1, IMU_ADDR<<1, MemsTxBuf, 1 , 100);
		int retval2 = HAL_I2C_Master_Receive_DMA(&hi2c1, IMU_ADDR<<1, destination , size );
		if( retval != HAL_OK)
		{
			//asm("bkpt");
		}
		if( retval2 != HAL_OK)
		{
			//asm("bkpt");
		}
	}
}

void getSensorValuesSlow()	//kiolvassa a gyorsulásmérõ regisztereit.
{
	AccX = i2c_read_byte(ACCEL_XOUT_H)<<8;
	AccX += i2c_read_byte(ACCEL_XOUT_L);
	AccY = i2c_read_byte(ACCEL_YOUT_H)<<8;
	AccY += i2c_read_byte(ACCEL_YOUT_L);
	AccZ = i2c_read_byte(ACCEL_ZOUT_H)<<8;
	AccZ += i2c_read_byte(ACCEL_ZOUT_L);

	GyroX = i2c_read_byte(GYRO_XOUT_H)<<8;
	GyroX += i2c_read_byte(GYRO_XOUT_L);
	GyroY = i2c_read_byte(GYRO_YOUT_H)<<8;
	GyroY += i2c_read_byte(GYRO_YOUT_L);
	GyroZ = i2c_read_byte(GYRO_ZOUT_H)<<8;
	GyroZ += i2c_read_byte(GYRO_ZOUT_L);
}

void getSensorValues()	//kiolvassa a gyorsulásmérõ regisztereit
{
	i2c_read_burst(ACCEL_XOUT_H , AccDataRaw ,  6);
	AccX = (AccDataRaw[0] << 8)  + AccDataRaw[1];
	AccY = (AccDataRaw[2] << 8)  + AccDataRaw[3];
	AccZ = (AccDataRaw[4] << 8)  + AccDataRaw[5];
	i2c_read_burst(GYRO_XOUT_H , GyroDataRaw ,  6);
	GyroX = (GyroDataRaw[0] << 8)  + GyroDataRaw[1];
	GyroY = (GyroDataRaw[2] << 8)  + GyroDataRaw[3];
	GyroZ = (GyroDataRaw[4] << 8)  + GyroDataRaw[5];
}

void ProcessData()
{
	double acc_total_vector;
	float angle_roll_acc, angle_pitch_acc;
	static float angle_pitch = 0;
	static float angle_roll = 0;
	static float angle_yaw = 0;

	//Subtract the offset values from the raw gyro values
	GyroX -= GyroXCalib;
	GyroY -= GyroYCalib;
	GyroZ -= GyroZCalib;

	//Gyro angle calculations. Integrate angular velocity by sampling time to get angle.
	angle_pitch += deltaTsec * GyroX / GYRO_SENS_FS500;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
	angle_roll += deltaTsec * GyroY / GYRO_SENS_FS500;                                    //Calculate the traveled roll angle and add this to the angle_roll variable
	angle_yaw += deltaTsec * GyroZ / GYRO_SENS_FS500;

	//Gyro compensate with yaw.
	angle_pitch += angle_roll * sin((float)GyroZ * (deltaTsec / GYRO_SENS_FS500) * (PI / 180.0));               //If the IMU has yawed transfer the roll angle to the pitch angel
	angle_roll -= angle_pitch * sin((float)GyroZ * (deltaTsec / GYRO_SENS_FS500) * (PI / 180.0));               //If the IMU has yawed transfer the pitch angle to the roll angel

	//Calculate angles by accelerometer measurements
	double acc_vector_squaresum = ((float)AccX * (float)AccX) + ((float)AccY * (float)AccY) + ((float)AccZ * (float)AccZ);
	acc_total_vector = sqrt(acc_vector_squaresum);		//Calculate the total accelerometer vector

	//calculate pitch and roll angles, from the total gravity vector
	angle_pitch_acc = asin((float)AccY/acc_total_vector)* (180/PI);       //Calculate the pitch angle and convert to degrees
	angle_roll_acc = asin((float)AccX/acc_total_vector)* (-180/PI);       //Calculate the roll angle and convert to degrees

	angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
	angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

	//use complementary filter to weigh gyro and accelerometer data
	angle_pitch = angle_pitch * 0.98 + angle_pitch_acc * 0.02;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle with complementary filter
	angle_roll = angle_roll * 0.98 + angle_roll_acc * 0.02;        //Correct the drift of the gyro roll angle with the accelerometer roll angle with complementary filter

	AngularX = angle_roll;
	AngularY = angle_pitch;
	AngularZ = angle_yaw;
}

void ReadMEMSTask(void const * argument)
{
	osDelay(200);
	IMU_Init();
	IMU_Calibrate();
	while(1)
	{
		getSensorValuesSlow();
		ProcessData();

		osDelay(deltaTsec*1000);
	}
	osThreadTerminate(NULL);
}



/* USER CODE BEGIN 1 */
/**
1. Disable the I2C peripheral by clearing the PE bit in I2Cx_CR1 register.
2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level
(Write 1 to GPIOx_ODR).
3. Check SCL and SDA High level in GPIOx_IDR.
4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to
GPIOx_ODR).
5. Check SDA Low level in GPIOx_IDR.
6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to
GPIOx_ODR).
7. Check SCL Low level in GPIOx_IDR.
8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to
GPIOx_ODR).
9. Check SCL High level in GPIOx_IDR.
10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to
GPIOx_ODR).
11. Check SDA High level in GPIOx_IDR.
12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.
13. Set SWRST bit in I2Cx_CR1 register.
14. Clear SWRST bit in I2Cx_CR1 register.
15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register.
**/
void HAL_GPIO_WRITE_ODR(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  /* Check the parameters */
  assert_param(IS_GPIO_PIN(GPIO_Pin));

  GPIOx->ODR |= GPIO_Pin;
}
void HAL_I2C_ClearBusyFlagErrata_2_14_7(I2C_HandleTypeDef *hi2c) {

    static uint8_t resetTried = 0;
    if (resetTried == 1) {
        return ;
    }
    uint32_t SDA_PIN = GPIO_PIN_7;
    uint32_t SCL_PIN = GPIO_PIN_6;
    GPIO_InitTypeDef GPIO_InitStruct;

    // 1
    __HAL_I2C_DISABLE(hi2c);

    // 2
    GPIO_InitStruct.Pin = SDA_PIN|SCL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WRITE_ODR(GPIOB, SDA_PIN);
    HAL_GPIO_WRITE_ODR(GPIOB, SCL_PIN);

    // 3
    GPIO_PinState pinState;
    if (HAL_GPIO_ReadPin(GPIOB, SDA_PIN) == GPIO_PIN_RESET) {
        for(;;){}
    }
    if (HAL_GPIO_ReadPin(GPIOB, SCL_PIN) == GPIO_PIN_RESET) {
        for(;;){}
    }

    // 4
    GPIO_InitStruct.Pin = SDA_PIN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_TogglePin(GPIOB, SDA_PIN);

    // 5
    if (HAL_GPIO_ReadPin(GPIOB, SDA_PIN) == GPIO_PIN_SET) {
        for(;;){}
    }

    // 6
    GPIO_InitStruct.Pin = SCL_PIN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_TogglePin(GPIOB, SCL_PIN);

    // 7
    if (HAL_GPIO_ReadPin(GPIOB, SCL_PIN) == GPIO_PIN_SET) {
        for(;;){}
    }

    // 8
    GPIO_InitStruct.Pin = SDA_PIN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WRITE_ODR(GPIOB, SDA_PIN);

    // 9
    if (HAL_GPIO_ReadPin(GPIOB, SDA_PIN) == GPIO_PIN_RESET) {
        for(;;){}
    }

    // 10
    GPIO_InitStruct.Pin = SCL_PIN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WRITE_ODR(GPIOB, SCL_PIN);

    // 11
    if (HAL_GPIO_ReadPin(GPIOB, SCL_PIN) == GPIO_PIN_RESET) {
        for(;;){}
    }

    // 12
    GPIO_InitStruct.Pin = SDA_PIN|SCL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

   // 13
   hi2c->Instance->CR1 |= I2C_CR1_SWRST;

   // 14
   hi2c->Instance->CR1 ^= I2C_CR1_SWRST;

   // 15
   __HAL_I2C_ENABLE(hi2c);

   resetTried = 1;
}



