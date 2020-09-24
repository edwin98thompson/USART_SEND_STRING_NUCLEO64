/*
 * STM32_RCC_driver.h
 *
 *  Created on: 23 Sep 2020
 *      Author: edwin.thompson
 */

#ifndef INC_STM32_RCC_DRIVER_H_
#define INC_STM32_RCC_DRIVER_H_

#include "STM32_RCC_driver.h"

#include "STM_NUCLEO_64.h"
#include "STM32_GPIO_driver.h"

uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);
uint8_t RCC_GetPLLOutputCLK(void);

#endif /* INC_STM32_RCC_DRIVER_H_ */
