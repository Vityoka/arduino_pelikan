/**
  ******************************************************************************
  * File Name          : main.hpp
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define Logic_battery_Pin GPIO_PIN_0
#define Logic_battery_GPIO_Port GPIOC
#define RPI_MOSI_Pin GPIO_PIN_1
#define RPI_MOSI_GPIO_Port GPIOC
#define RPI_MISO_Pin GPIO_PIN_2
#define RPI_MISO_GPIO_Port GPIOC
#define Motor_curr_Pin GPIO_PIN_3
#define Motor_curr_GPIO_Port GPIOC
#define HMI_RX_Pin GPIO_PIN_0
#define HMI_RX_GPIO_Port GPIOA
#define HMI_TX_Pin GPIO_PIN_1
#define HMI_TX_GPIO_Port GPIOA
#define STARTER_RX_Pin GPIO_PIN_2
#define STARTER_RX_GPIO_Port GPIOA
#define STARTER_TX_Pin GPIO_PIN_3
#define STARTER_TX_GPIO_Port GPIOA
#define IR_RightS_Pin GPIO_PIN_4
#define IR_RightS_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define IR_FrontS_Pin GPIO_PIN_6
#define IR_FrontS_GPIO_Port GPIOA
#define IR_FrontL_Pin GPIO_PIN_7
#define IR_FrontL_GPIO_Port GPIOA
#define IR_LeftS_Pin GPIO_PIN_4
#define IR_LeftS_GPIO_Port GPIOC
#define VonalGetData_Pin GPIO_PIN_5
#define VonalGetData_GPIO_Port GPIOC
#define Motor_battery_Pin GPIO_PIN_0
#define Motor_battery_GPIO_Port GPIOB
#define RPI_NSS_Pin GPIO_PIN_12
#define RPI_NSS_GPIO_Port GPIOB
#define RPI_SCK_Pin GPIO_PIN_13
#define RPI_SCK_GPIO_Port GPIOB
#define RC_PWM1_in_Pin GPIO_PIN_14
#define RC_PWM1_in_GPIO_Port GPIOB
#define Motor_PWM1_Pin GPIO_PIN_6
#define Motor_PWM1_GPIO_Port GPIOC
#define Motor_PWM2_Pin GPIO_PIN_7
#define Motor_PWM2_GPIO_Port GPIOC
#define IR_Data_Pin GPIO_PIN_8
#define IR_Data_GPIO_Port GPIOC
#define Servo_PWM_Pin GPIO_PIN_8
#define Servo_PWM_GPIO_Port GPIOA
#define BT_RX_Pin GPIO_PIN_9
#define BT_RX_GPIO_Port GPIOA
#define BT_TX_Pin GPIO_PIN_10
#define BT_TX_GPIO_Port GPIOA
#define RollingInfraServo_Pin GPIO_PIN_11
#define RollingInfraServo_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define Vonal_NSS_Pin GPIO_PIN_15
#define Vonal_NSS_GPIO_Port GPIOA
#define Vonal_SCK_Pin GPIO_PIN_10
#define Vonal_SCK_GPIO_Port GPIOC
#define Vonal_MISO_Pin GPIO_PIN_11
#define Vonal_MISO_GPIO_Port GPIOC
#define Vonal_MOSI_Pin GPIO_PIN_12
#define Vonal_MOSI_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define ENC_A_Pin GPIO_PIN_8
#define ENC_A_GPIO_Port GPIOB
#define ENC_B_Pin GPIO_PIN_9
#define ENC_B_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
#define NUM_OF_TIMERS 1
void SetTasks();
void InitTimers();
/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
