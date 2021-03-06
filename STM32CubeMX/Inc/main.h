/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PC13_Pin GPIO_PIN_13
#define PC13_GPIO_Port GPIOC
#define VNH2_SP30_PWM_Pin GPIO_PIN_0
#define VNH2_SP30_PWM_GPIO_Port GPIOA
#define STOP_IN_PA3_Pin GPIO_PIN_3
#define STOP_IN_PA3_GPIO_Port GPIOA
#define TPS_ADC_Pin GPIO_PIN_0
#define TPS_ADC_GPIO_Port GPIOB
#define SWITCH_ADC_Pin GPIO_PIN_1
#define SWITCH_ADC_GPIO_Port GPIOB
#define PB10_OUT_SOLENOID_Pin GPIO_PIN_10
#define PB10_OUT_SOLENOID_GPIO_Port GPIOB
#define RPM_EXTI11_Pin GPIO_PIN_11
#define RPM_EXTI11_GPIO_Port GPIOB
#define RPM_EXTI11_EXTI_IRQn EXTI15_10_IRQn
#define OD_IN_LIGHT_PB12_Pin GPIO_PIN_12
#define OD_IN_LIGHT_PB12_GPIO_Port GPIOB
#define OD_OUT_BUTTON_PB13_Pin GPIO_PIN_13
#define OD_OUT_BUTTON_PB13_GPIO_Port GPIOB
#define IDLE_IN_PB14_Pin GPIO_PIN_14
#define IDLE_IN_PB14_GPIO_Port GPIOB
#define CRUISE_LAMP_OUT_PB15_Pin GPIO_PIN_15
#define CRUISE_LAMP_OUT_PB15_GPIO_Port GPIOB
#define D_IN_PA8_Pin GPIO_PIN_8
#define D_IN_PA8_GPIO_Port GPIOA
#define TIM3_CH2_SPD_Pin GPIO_PIN_4
#define TIM3_CH2_SPD_GPIO_Port GPIOB
#define TB6612FNG_STB_Pin GPIO_PIN_5
#define TB6612FNG_STB_GPIO_Port GPIOB
#define VNH2_SP30_INA_Pin GPIO_PIN_8
#define VNH2_SP30_INA_GPIO_Port GPIOB
#define VNH2_SP30_INB_Pin GPIO_PIN_9
#define VNH2_SP30_INB_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
