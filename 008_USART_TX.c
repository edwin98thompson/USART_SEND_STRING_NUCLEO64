/*
 * 008_USART_TX.c
 *
 *  Created on: 23 Sep 2020
 *      Author: edwin.thompson
 */


#include<stdio.h>
#include<string.h>

#include "../Drivers/inc/STM_NUCLEO_64.h"
#include "../Drivers/inc/STM32_GPIO_driver.h"
#include "../Drivers/inc/STM32_USART_driver.h"


//AF7
//USART2 CTS = PA0
//USART2 RTS = PA1
//USART2 TX = PB6
//USART2 RX = PB7
//USART2 CL = PA4

USART_Handle_t usart1_handle;

char msg[1024] = "usart test string..... \n\r";

//extern void initialise_monitor_handles(void);


void delay(void)
{
	for(uint32_t i = 0 ; i < 500000 ; i ++);
}

void USART2_Init(void)
{
	usart1_handle.pUSARTx = USART1;
	usart1_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	usart1_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart1_handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	usart1_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	usart1_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	usart1_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART_Init(&usart1_handle);
}

void USART2_GPIOInit(void)
{
	GPIO_Handle_t usart_gpios;

	usart_gpios.pGPIOx = GPIOB;
	usart_gpios.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	usart_gpios.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	usart_gpios.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_PIN_PU;
	usart_gpios.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	usart_gpios.GPIO_PinConfig.GPIO_PinAltFunMode = 7;

	//USART TX
	usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&usart_gpios);

	//USART RX
	usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&usart_gpios);

}

void GPIO_Button_Init(void)
{
	GPIO_Handle_t gpioBtn, GpioLed;

	gpioBtn.pGPIOx = GPIOC;
	gpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;	// config stuff for user button
	gpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	gpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioBtn.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_NO_PUPD;

	GPIO_PER_CLK_Control(GPIOC, ENABLE); //enasble clock
	GPIO_Init(&gpioBtn); //initialise

	//this is led gpio configuration
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_NO_PUPD;

	GPIO_PER_CLK_Control(GPIOA,ENABLE);

	GPIO_Init(&GpioLed);
}

int main(void){

	//initialise_monitor_handles();

	GPIO_Button_Init();

	USART2_GPIOInit();

	USART2_Init();

	USART_PeripheralControl(USART1, ENABLE);

	while(1)
	{

		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);

		delay();
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
		USART_SendData(&usart1_handle, (uint8_t*)msg,strlen(msg));
	}


	return 0;
}
