/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "ILI9225.h"
//#include"Image.h"
#include "Menu.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
enum Shifts //Сдвиги для обработки кнопок
{
	LB = 0,
	LL,
	LU,
	LR,
	RB,
	RL,
	RU,
	RR,
	OK
};

typedef union
{
   uint8_t IsButtonPressedRxBuff[2];
   uint16_t IsButtonPressedRx;
}FlagUnion;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void Post_Processing_Buttons();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CHRG_Pin GPIO_PIN_13
#define CHRG_GPIO_Port GPIOC
#define CHRG_EXTI_IRQn EXTI15_10_IRQn
#define Battt_Voltage_Pin GPIO_PIN_0
#define Battt_Voltage_GPIO_Port GPIOA
#define LR_Pin GPIO_PIN_3
#define LR_GPIO_Port GPIOA
#define LR_EXTI_IRQn EXTI3_IRQn
#define RB_Pin GPIO_PIN_4
#define RB_GPIO_Port GPIOA
#define RB_EXTI_IRQn EXTI4_IRQn
#define RL_Pin GPIO_PIN_5
#define RL_GPIO_Port GPIOA
#define RL_EXTI_IRQn EXTI9_5_IRQn
#define RU_Pin GPIO_PIN_6
#define RU_GPIO_Port GPIOA
#define RU_EXTI_IRQn EXTI9_5_IRQn
#define RR_Pin GPIO_PIN_7
#define RR_GPIO_Port GPIOA
#define RR_EXTI_IRQn EXTI9_5_IRQn
#define LB_Pin GPIO_PIN_0
#define LB_GPIO_Port GPIOB
#define LB_EXTI_IRQn EXTI0_IRQn
#define LL_Pin GPIO_PIN_1
#define LL_GPIO_Port GPIOB
#define LL_EXTI_IRQn EXTI1_IRQn
#define LU_Pin GPIO_PIN_2
#define LU_GPIO_Port GPIOB
#define LU_EXTI_IRQn EXTI2_IRQn
#define OK_Pin GPIO_PIN_8
#define OK_GPIO_Port GPIOA
#define OK_EXTI_IRQn EXTI9_5_IRQn
#define LCD_DC_Pin GPIO_PIN_5
#define LCD_DC_GPIO_Port GPIOB
#define LCD_RST_Pin GPIO_PIN_6
#define LCD_RST_GPIO_Port GPIOB
#define LCD_CS_Pin GPIO_PIN_7
#define LCD_CS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
