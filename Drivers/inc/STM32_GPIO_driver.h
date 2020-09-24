/*
 * STM32_GPIO_driver.h
 *
 *  Created on: Sep 10, 2020
 *      Author: edwin.thompson
 */

#ifndef INC_STM32_GPIO_DRIVER_H_

#define INC_STM32_GPIO_DRIVER_H_

#include "STM_NUCLEO_64.h"


typedef struct
{
	uint8_t GPIO_PinNumber;		//pin numbers listed in //GPIO pin numbers
	uint8_t GPIO_PinMode;		//Possible pin modes listed in //GPIO pin possible modes
	uint8_t GPIO_PinSpeed;		//possible speeds listed in //GPIO possible output speeds
	uint8_t GPIO_PinPUPDControl;//settings listed in //GPIO PIN  up/ pull down settings
	uint8_t GPIO_PinOPType;		//types listed in //GPIO pin possible output types
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;


// handle structure for GPIO
typedef struct
{
	GPIO_RegDef_t *pGPIOx; 				//holds base address of gpio port to which pin belongs
	GPIO_PinConfig_t GPIO_PinConfig; 	//holds gpio pin config settings

}GPIO_Handle_t;

//GPIO pin numbers
#define GPIO_PIN_NO_0			0
#define GPIO_PIN_NO_1			1
#define GPIO_PIN_NO_2			2
#define GPIO_PIN_NO_3			3
#define GPIO_PIN_NO_4			4
#define GPIO_PIN_NO_5			5
#define GPIO_PIN_NO_6			6
#define GPIO_PIN_NO_7			7
#define GPIO_PIN_NO_8			8
#define GPIO_PIN_NO_9			9
#define GPIO_PIN_NO_10			10
#define GPIO_PIN_NO_11			11
#define GPIO_PIN_NO_12			12
#define GPIO_PIN_NO_13			13
#define GPIO_PIN_NO_14			14
#define GPIO_PIN_NO_15			15

//GPIO pin possible modes
#define GPIO_MODE_INPUT 		0
#define GPIO_MODE_OUTPUT 		1
#define GPIO_MODE_ALTFN			2
#define GPIO_MODE_ANALOG		3
#define GPIO_MODE_IT_FT			4  //interrupt falling edge trigger
#define GPIO_MODE_IT_RT			5  //interrupt rising edge trigger
#define GPIO_MODE_IT_RFT		6  //interrupt rise/fall edge trigger

//GPIO pin possible output types
#define GPIO_OP_TYPE_PP			0  //push/pull
#define GPIO_OP_TYPE_OD			1  //open drain

//GPIO possible output speeds
#define GPIO_SPEED_LOW 			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPIO_SPEED_HIGH			3

//GPIO PIN  up/ pull down settings
#define GPIO_NO_PUPD			0
#define GPIO_PIN_PU				1
#define GPIO_PIN_PD				2


/************************************************************************************************
 * 										APIs supported by this driver
 * 									Check func defs for more info on APIs
 ************************************************************************************************/
//periph clock setup
void GPIO_PER_CLK_Control(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi);

//init and deinit
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_Deinit(GPIO_RegDef_t *pGPIOx);

//data read write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t  GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

//IRQ config/handle
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

//func prototypes also go in corresponding driver.c file to be implemented


#endif /* INC_STM32_GPIO_DRIVER_H_ */

