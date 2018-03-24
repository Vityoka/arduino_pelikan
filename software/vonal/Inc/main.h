/**
  ******************************************************************************
  * File Name          : main.hpp
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

#define EN_sensors_Pin GPIO_PIN_1
#define EN_sensors_GPIO_Port GPIOB
#define Select0_Pin GPIO_PIN_2
#define Select0_GPIO_Port GPIOB
#define Select1_Pin GPIO_PIN_10
#define Select1_GPIO_Port GPIOB
#define Select2_Pin GPIO_PIN_11
#define Select2_GPIO_Port GPIOB
#define SPI2_Main_SS_Pin GPIO_PIN_12
#define SPI2_Main_SS_GPIO_Port GPIOB
#define SPI2_Main_SCK_Pin GPIO_PIN_13
#define SPI2_Main_SCK_GPIO_Port GPIOB
#define SPI2_Main_MISO_Pin GPIO_PIN_14
#define SPI2_Main_MISO_GPIO_Port GPIOB
#define SPI2_Main_MOSI_Pin GPIO_PIN_15
#define SPI2_Main_MOSI_GPIO_Port GPIOB
#define fbLED_OE_Pin GPIO_PIN_8
#define fbLED_OE_GPIO_Port GPIOA
#define infra_OE_Pin GPIO_PIN_11
#define infra_OE_GPIO_Port GPIOA
#define infra_LE_Pin GPIO_PIN_12
#define infra_LE_GPIO_Port GPIOA
#define SPI1_infra_SCK_Pin GPIO_PIN_3
#define SPI1_infra_SCK_GPIO_Port GPIOB
#define SPI1_infra_SDO_Pin GPIO_PIN_5
#define SPI1_infra_SDO_GPIO_Port GPIOB
#define fbLED_SCK_Pin GPIO_PIN_6
#define fbLED_SCK_GPIO_Port GPIOB
#define fbLED_SDO_Pin GPIO_PIN_7
#define fbLED_SDO_GPIO_Port GPIOB
#define fbLED_LE_Pin GPIO_PIN_8
#define fbLED_LE_GPIO_Port GPIOB
#define MAIN_NOTIFY_Pin GPIO_PIN_9
#define MAIN_NOTIFY_GPIO_Port GPIOA
#define MAIN_WAITDATA_Pin GPIO_PIN_10
#define MAIN_WAITDATA_GPIO_Port GPIOA

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

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
