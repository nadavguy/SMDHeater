/*
 * PushButtonAgent.c
 *
 *  Created on: Jan 30, 2024
 *      Author: user
 */

#include "main.h"

uint32_t lastLeftButtonPressed = 0;
uint32_t lastRightButtonPressed = 0;

int32_t setPointValue[2] = {25, 25};

void checkButtons(void)
{
	//Left Side
	if ( (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET) && (HAL_GetTick() - lastLeftButtonPressed >= 100) )
	{
		lastLeftButtonPressed = HAL_GetTick();
		int32_t localValue = setPointValue[0] - 5;
		if (localValue < 25)
		{
			localValue = 25;
		}
		setPointValue[0] = localValue;
		newSetPoint[0] = true;
	}
	else if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_RESET) && (HAL_GetTick() - lastLeftButtonPressed >= 100))
	{
		lastLeftButtonPressed = HAL_GetTick();
		int32_t localValue = setPointValue[0] + 5;
		if (localValue > 200)
		{
			localValue = 200;
		}
		setPointValue[0] = localValue;
		newSetPoint[0] = true;
	}

	//Right Side
	if ( (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET) && (HAL_GetTick() - lastRightButtonPressed >= 100) )
	{
		lastRightButtonPressed = HAL_GetTick();
		int32_t localValue = setPointValue[1] - 5;
		if (localValue < 25)
		{
			localValue = 25;
		}
		setPointValue[1] = localValue;
		newSetPoint[1] = true;
	}
	else if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_RESET) && (HAL_GetTick() - lastRightButtonPressed >= 100))
	{
		lastRightButtonPressed = HAL_GetTick();
		int32_t localValue = setPointValue[1] + 5;
		if (localValue > 200)
		{
			localValue = 200;
		}
		setPointValue[1] = localValue;
		newSetPoint[1] = true;
	}
}
