/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "MenuHelper.h"
#include "FrameHelper.h"
#include "ItemHelper.h"
#include "PopupHelper.h"
#include "fonts.h"
#include "ScreenAgent.h"
#include "GeneralMath.h"
#include "PushButtonAgent.h"
#include "ThermistorAgent.h"


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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ADC1_IN0_LeftSensor_Pin GPIO_PIN_0
#define ADC1_IN0_LeftSensor_GPIO_Port GPIOA
#define ADC1_IN1_MiddleSensor_Pin GPIO_PIN_1
#define ADC1_IN1_MiddleSensor_GPIO_Port GPIOA
#define ADC1_IN2_RightSensor_Pin GPIO_PIN_2
#define ADC1_IN2_RightSensor_GPIO_Port GPIOA
#define GPIO_Screen_CS_Pin GPIO_PIN_4
#define GPIO_Screen_CS_GPIO_Port GPIOA
#define SPI1_SCK_Screen_Pin GPIO_PIN_5
#define SPI1_SCK_Screen_GPIO_Port GPIOA
#define GPIO_Screen_DC_Pin GPIO_PIN_6
#define GPIO_Screen_DC_GPIO_Port GPIOA
#define SPI1_MOSI_Screen_Pin GPIO_PIN_7
#define SPI1_MOSI_Screen_GPIO_Port GPIOA
#define GPIO_Left_Heater_Down_Pin GPIO_PIN_12
#define GPIO_Left_Heater_Down_GPIO_Port GPIOB
#define GPIO_Left_Heater_Up_Pin GPIO_PIN_13
#define GPIO_Left_Heater_Up_GPIO_Port GPIOB
#define GPIO_Right_Heater_Down_Pin GPIO_PIN_14
#define GPIO_Right_Heater_Down_GPIO_Port GPIOB
#define GPIO_Right_Heater_Up_Pin GPIO_PIN_15
#define GPIO_Right_Heater_Up_GPIO_Port GPIOB
#define TIM1_CH1_Heater1PWM_Pin GPIO_PIN_8
#define TIM1_CH1_Heater1PWM_GPIO_Port GPIOA
#define TIM1_CH2_Heater2PWM_Pin GPIO_PIN_9
#define TIM1_CH2_Heater2PWM_GPIO_Port GPIOA
#define TIM1_CH3_Screen_BL_Pin GPIO_PIN_10
#define TIM1_CH3_Screen_BL_GPIO_Port GPIOA
#define GPIO_Screen_Reset_Pin GPIO_PIN_11
#define GPIO_Screen_Reset_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
extern float versionID;
extern float buildID;

extern tCURSOR_DATA currentCursorPosition;

extern bool newThermistorMeasurementAvailable;
extern uint32_t timer1;

//extern KalmanFilter leftThermistor;
//extern KalmanFilter middleThermistor;
//extern KalmanFilter rightThermistor;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
