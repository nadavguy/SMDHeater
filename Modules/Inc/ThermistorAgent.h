/*
 * ThermistorAgent.h
 *
 *  Created on: Jan 23, 2024
 *      Author: user
 */

#ifndef INC_THERMISTORAGENT_H_
#define INC_THERMISTORAGENT_H_

#include "stdint.h"

extern uint32_t thermistors[3];
extern float thermistorsVoltage[3];
extern float thermistorsResistance[3];
extern int32_t thermistorsTemperature[3];
extern float resistors[3];

extern void convertVoltageToTemperature(void);
#endif /* INC_THERMISTORAGENT_H_ */
