/*
 * STM32_GPIO_driver.c
 *
 *  Created on: Sep 10, 2020
 *      Author: edwin.thompson
 */


#include "../inc/STM32_GPIO_driver.h"

/**
 * GPIO_PER_CLK_Control
 *
 * function to enable/disable peripheral clock for general purpose i/o
 *
 *
 * @param[in]: GPIO_RegDef_t *pGPIOx : base address of GPIO peripheral
 * @param[in]: uint8_t EnorDi : enable/ disable macros
 * @return : NONE
 */
void GPIO_PER_CLK_Control(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}
}

//init and deinit

/**
 * GPIO_Init
 *
 * function to initialise general purpose IO pins
 *
 *
 * @param[in]: GPIO_HANDLE : contains base address and config settings
 * @param[in]:
 * @return : NONE
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)//struct pointer passed in, settings configured from this idk
{
	uint32_t temp = 0; //var for temp register

	//enable peripheral clock

	GPIO_PER_CLK_Control(pGPIOHandle->pGPIOx, ENABLE);

	//config pin mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//non interrupt
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
		//2bit fields per pin
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//clearing bitfields
		pGPIOHandle->pGPIOx->MODER |= temp;//use bitwise or to assign register
	}
	else
	{
		//interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			// config FTSR
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			//clear corresponding RTSR bit
			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//config RTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			//clear corresponding FTSR bit
			EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			// config RTSR and FTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		}

		// config GPIO port select is SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;  //potential source of error refer to ref manuel and video 112
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASE_ADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << ( temp2 * 4);

		// enable EXTI interrupt delivery using IMR
		EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	}
	temp = 0;

	//config speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//clearing bitfields
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	//config pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPUPDControl << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//clearing bitfields
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	//config optype
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//clearing bitfields
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	//config alt func
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << ( 4 * temp2 ));//clear bit fields
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * temp2 ));

	}
}

/**
 * GPIO_DEInit
 *
 * function to DEinitialise general purpose IO pins
 *
 *
 * @param[in]: GPIO_RegDef_t *pGPIOx : base address of GPIO peripheral
 * @param[in]:
 * @return : NONE
 */
void GPIO_Deinit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}

//data read write

/**
 * GPIO_ReadFromInputPin
 *
 * function to read from input pin
 *
 *
 * @param[in]: GPIO_RegDef_t *pGPIOx : base address of GPIO peripheral
 * @param[in]: PinNumber
 * @return : uint8_t :0 or 1
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);//input data reg shifted by value of pin number to get to lsb of reg
	return value;
}

/**
 * GPIO_ReadFromInputPort
 *
 * function to read from input port
 *
 *
 * @param[in]: GPIO_RegDef_t *pGPIOx : base address of GPIO peripheral
 * @param[in]:
 * @return : uint16_t : port data
 */
uint16_t  GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint8_t)pGPIOx->IDR;//return entire port
	return value;
}

/**
 * GPIO_WriteToOutputPin
 *
 * function to write to output pin
 *
 *
 * @param[in]: GPIO_RegDef_t *pGPIOx : base address of GPIO peripheral
 * @param[in]: PinNumber
 * @param[in]: value : value to write
 * @return : NONE
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		//write 1 to output data reg at the bit field corresponding to pin number
		pGPIOx->ODR |= ( 1 << PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~( 1 << PinNumber);
	}
}

/**
 * GPIO_WriteToOutputPort
 *
 * function to write to output port
 *
 *
 * @param[in]: GPIO_RegDef_t *pGPIOx : base address of GPIO peripheral
 * @param[in]:
 * @param[in]: value : value to write
 * @return : NONE
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t Value)
{
	pGPIOx->ODR = Value;
}

/**
 * GPIO_ToggleOutputPin
 *
 * function to toggle output pin
 *
 *
 * @param[in]: GPIO_RegDef_t *pGPIOx : base address of GPIO peripheral
 * @param[in]: PinNumber
 * @param[in]:
 * @return : NONE
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

//IRQ config/handle

/**
 * IRQ_CONFIG
 *
 * function to config IRQ
 *
 *
 * @param[in]: IRQNumber
 * @param[in]: IRQPriority
 * @param[in]: EnorDi enable or disable
 * @return : NONE
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//prog iser0 reg
			*NVIC_ISER0 |= ( 1 << IRQNumber );
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//prog iser1 reg
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//prog iser2 reg
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			//prog icer0 reg
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//prog icer1 reg
			*NVIC_ISER1 |= ( 1 << IRQNumber % 32 );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//prog icer2 reg
			*NVIC_ISER2 |= ( 1 << IRQNumber % 64 );
		}
	}
}

/**
 * IRQ_PRIORITY_CONFIG
 *
 * function to config IRQ priority
 *
 *
 * @param[in]: IRQNumber
 * @param[in]: IRQPriority
 * @return : NONE
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	// first find IPR register (priority)
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED );

	*(NVIC_PR_BASE_ADDR + iprx ) |= ( IRQPriority << shift_amount );
}

/**
 * IRQ_Handle
 *
 * function to HANDLE IRQ
 *
 *
 * @param[in]: PinNumber

 * @return : NONE
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the exti pr register
	if(EXTI->PR & ( 1 << PinNumber))
	{
		//clear pending register
		EXTI->PR |= ( 1 << PinNumber);
	}

}
