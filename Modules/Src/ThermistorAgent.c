/*
 * ThermistorAgent.c
 *
 *  Created on: Jan 23, 2024
 *      Author: user
 */

#include "adc.h"
#include "stdint.h"

uint32_t thermistors[3] = {0};
float thermistorsVoltage[3] = {0};
float thermistorsResistance[3] = {0};
float thermistorsTemperature[3] = {0};
float filteredThermistorsTemperature[3] = {0};
float resistors[3] = {4700.0, 4700.0, 4700.0};

float resitanceTable[36] = { 100.0, 81.0, 65.65, 53.5, 43.78, 35.8999, 29.99, 25.0, 20.9, 17.55, 14.76, 12.54, 10.66, 9.1, 7.784, 6.71, 5.85, 5.07, 4.41, 3.85, 3.34, 2.94, 2.58,
		2.27, 2.00, 1.77, 1.58, 1.41, 1.259, 1.122, 0.997, 0.8960, 0.7970, 0.7190, 0.6430, 0.5820};
uint32_t  resitanceTableIndex[36] = { 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115, 120, 125, 130, 135, 140, 145, 150, 155, 160, 165, 170, 175, 180, 185, 190, 195, 200};

void InitADCs(void)
{
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&thermistors, 3);
}

void convertVoltageToTemperature(void)
{
	for (int i = 0 ; i < 3 ; i++)
	{
		thermistorsResistance[i] = resistors[i] * thermistorsVoltage[i] / ( (3.3 - thermistorsVoltage[i]) * 1000);
		int n = 36;
		int localTableIndex = binarySearch(resitanceTable, 0, n - 1, thermistorsResistance[i]);
		if (localTableIndex >= 0)
		{
			thermistorsTemperature[i] = resitanceTableIndex[localTableIndex];
		}
		else if (localTableIndex == -1)
		{
			thermistorsTemperature[i] = -1;
		}
		else if (localTableIndex == 999)
		{
			thermistorsTemperature[i] = 999;
		}
		else
		{

		}
	}
}
