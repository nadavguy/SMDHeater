/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "KalmanFilter.h"
#include "PIDController.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
KalmanFilter leftThermistorKF(0.25, 1, 25, 25);
KalmanFilter middleThermistorKF(0.25, 1, 25, 25);
KalmanFilter rightThermistorKF(0.25, 1, 25, 25);

PIDController leftThermistorPID(0.001, 10, 1, 25);
PIDController middleThermistorPID(0.001, 10, 1, 25);
PIDController rightThermistorPID(0.001, 10, 1, 25);

bool newThermistorMeasurementAvailable = false;
uint32_t timer1 = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float versionID = 1.000;
float buildID = 1.030;

tCURSOR_DATA currentCursorPosition;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  TIM1->CCR3 = 20000;
  screenInit();
  screenClear();
  renderCompleteFrame = true;
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&thermistors, 3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  checkButtons();
	  leftThermistorPID.setSP(setPointValue[0]);
	  middleThermistorPID.setSP( (setPointValue[0] + setPointValue[1])/2.0);
	  rightThermistorPID.setSP(setPointValue[1]);

	  if (newThermistorMeasurementAvailable)
	  {
		  float localLeftThermistor = (float)leftThermistorKF.getFilteredValue(thermistorsTemperature[0]);
		  float localMiddleThermistor = (float)middleThermistorKF.getFilteredValue(thermistorsTemperature[1]);
		  float localRightThermistor = (float)rightThermistorKF.getFilteredValue(thermistorsTemperature[2]);

		  filteredThermistorsTemperature[0] = localLeftThermistor;
		  filteredThermistorsTemperature[1] = localMiddleThermistor;
		  filteredThermistorsTemperature[2] = localRightThermistor;

		  newThermistorMeasurementAvailable = false;
		  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&thermistors, 3);
	  }
	  leftThermistorPID.updateCycle((float)filteredThermistorsTemperature[0]);
	  middleThermistorPID.updateCycle((float)filteredThermistorsTemperature[1]);
	  rightThermistorPID.updateCycle((float)filteredThermistorsTemperature[2]);

	  uint32_t localLeftThermistorTim = (uint32_t)(TIM1->CCR1 + leftThermistorPID.getOutput());
	  uint32_t localRightThermistorTim = (uint32_t)(TIM1->CCR2 + rightThermistorPID.getOutput());

	  if (localLeftThermistorTim > 20000)
	  {
		  localLeftThermistorTim = 20000;
	  }

	  if (localLeftThermistorTim < 0)
	  {
		  localLeftThermistorTim = 0;
	  }

    if (localRightThermistorTim > 20000)
	  {
		  localRightThermistorTim = 20000;
	  }

	  if (localRightThermistorTim < 0)
	  {
		  localRightThermistorTim = 0;
	  }

	  TIM1->CCR1 = localLeftThermistorTim;
	  TIM1->CCR2 = localRightThermistorTim;

	  timer1 = localLeftThermistorTim;


	  screenUpdate(false);
	  displayNextFrame();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
