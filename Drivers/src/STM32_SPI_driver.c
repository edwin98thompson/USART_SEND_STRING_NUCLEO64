/*
 * STM32_SPI_driver.c
 *
 *  Created on: 15 Sep 2020
 *      Author: edwin.thompson
 */

#include "STM32_SPI_driver.h"

//helper function prototypes (not called by user application) private, not defined in spi_driver.h
//static keyword sets function as private
//more interrupt handling
static void spi_RXNE_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_OVR_ERR_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_TXE_interrupt_handle(SPI_Handle_t *pSPIHandle);

//periph clock setup

/**
 * SPI_PER_CLK_Control
 *
 * function to enable/disable peripheral clock for SPI peripheral
 *
 *
 * @param[in]: SPI_RegDef_t *pSPIx : base address of SPI peripheral
 * @param[in]: uint8_t EnorDi : enable/ disable macros
 * @return : NONE
 */
void SPI_PER_CLK_Control(SPI_RegDef_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
		else if(pSPIx == SPI4)
		{
			SPI4_PCLK_EN();
		}
	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
		else if(pSPIx == SPI4)
		{
			SPI4_PCLK_DI();
		}
	}
}

//init and deinit

/**
 * SPI_Init
 *
 * function to initialise SPI periphs
 *
 *
 * @param[in]: SPI_HANDLE : contains base address and config settings
 * @param[in]:
 * @return : NONE
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//periph clock enable
	SPI_PER_CLK_Control(pSPIHandle->pSPIx, ENABLE);

	// config spi CR1 register

	uint32_t tempreg = 0;

	//1. config device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. config bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FDUPLEX)
	{
		//clear bidirectional mode
		tempreg &= ~( 1 << SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HDUPLEX)
	{
		//set bidirectional mode
		tempreg |= ( 1 << SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//clear bidirectional mode
		tempreg &= ~( 1 << SPI_CR1_BIDIMODE);
		//rx only bit set
		tempreg |= ( 1 << SPI_CR1_RXONLY);
	}
	//3. config clock speed
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4. config DFF data frame format
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5. config CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6. config CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	//assignment of configured bitfields
	pSPIHandle->pSPIx->CR1 = tempreg;
}

/**
 * SPI_DEInit
 *
 * function to DEinitialise SPI periphs
 *
 *
 * @param[in]: SPI_RegDef_t *pSPIx: base address of SPI peripheral
 * @param[in]:
 * @return : NONE
 */
void SPI_Deinit(SPI_RegDef_t *pSPIx)
{

}

//Data send and receive

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/**
 * SPI_SEND_DATA
 *
 * function to send data via SPI
 *
 *
 * @param[in]: SPI_RegDef_t *pSPIx: base address of SPI peripheral
 * @param[in]: uint8_t *pTXBuffer: the data buffer to be sent
 * @param[in]:uint32_t Len: size of data
 * @return : uint8_t :0 or 1
 *  NOTE: This is a blocking call, wont leave func till all data sent
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTXBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		//2.  check the DFF bit in CR1
		if(pSPIx->CR1 & ( 1 << SPI_CR1_DFF))
		{
			//16 bit DFF
			//1. load data into DR
			pSPIx->DR = *((uint16_t*)pTXBuffer);
			Len--;
			Len--;
			(uint16_t*)pTXBuffer++;
		}
		else
		{
			//8 bit DFF
			pSPIx->DR = *pTXBuffer;
			Len--;
			pTXBuffer++;
		}
	}
}

/**
 * SPI_ReceiveData
 *
 * function to receive data via SPI
 *
 *
 * @param[in]: SPI_RegDef_t *pSPIx : base address of SPI peripheral
 * @param[in]: uint8_t *pRXBuffer : buffer of data being received
 * @param[in]: uint32_t Len: size of data
 * @return : NONE
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRXBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. wait until RXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		//2.  check the DFF bit in CR1
		if(pSPIx->CR1 & ( 1 << SPI_CR1_DFF))
		{
			//16 bit DFF
			//1. load data from DR to RX buffer
			*((uint16_t*)pRXBuffer) = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*)pRXBuffer++;
		}
		else
		{
			//8 bit DFF
			*((uint8_t*)pRXBuffer) = pSPIx->DR;
			Len--;
			pRXBuffer++;
		}
	}
}


//Data send and receive (interrupt mode)

/**
 * SPI_SendDataIT
 *
 * fn to send spi data using isr
 *
 *
 * @param[in]: SPI_Handle_t *pSPIHandle
 * @param[in]: uint8_t *pTXBuffer, data transmit buffer
 * @param[in]: uint32_t Len: size of data
 * @return : NONE
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTXBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TXState;

	if(state != SPI_BUSY_IN_TX)
	{
		//1. save tx buffer address, len info
		pSPIHandle->pTXBuffer = pTXBuffer;
		pSPIHandle->TXLen = Len;

		//2. mark state busy so no other operations occur/interfere
		pSPIHandle->TXState = SPI_BUSY_IN_TX;

		//3. enable TXIEIE control bit
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_TXEIE );
	}

	//4. data tx handled by ISR // to do
	return state;
}

/**
 * SPI_ReceiveDataIT
 *
 * fn to receive spi data using isr
 *
 *
 * @param[in]: SPI_Handle_t *pSPIHandle
 * @param[in]: uint8_t *pRXBuffer : buffer of data being received
 * @param[in]: uint32_t Len: size of data
 * @return : NONE
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRXBuffer, uint32_t Len)
{
	//to do
	uint8_t state = pSPIHandle->RXState;

	if(state != SPI_BUSY_IN_RX)
	{
		//1. save Rx buffer address, len info
		pSPIHandle->pRXBuffer = pRXBuffer;
		pSPIHandle->RXLen = Len;

		//2. mark state busy so no other operations occur/interfere
		pSPIHandle->RXState = SPI_BUSY_IN_RX;

		//3. enable RXNE control bit
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_RXNEIE );
	}

	//4. data rx handled by ISR // to do
	return state;


}


//IRQ Configuration and ISR Handling

/**
 * SPI_IRQ_CONFIG
 *
 * function to config IRQ for SPI
 *
 *
 * @param[in]: IRQNumber
 * @param[in]: EnorDi enable or disable
 * @return : NONE
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	/*//to implement and edit next time
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
	*/
}

/**
 * IRQ_PRIORITY_CONFIG
 *
 * function to config IRQ priority for spi
 *
 *
 * @param[in]: IRQNumber
 * @param[in]: IRQPriority
 * @return : NONE
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{

}

/**
 * IRQ_Handle
 *
 * function to HANDLE IRQ for spi
 *
 *
 * @param[in]: spi handle

 * @return : NONE
 */
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1, temp2;

	//first check TXE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_TXEIE);

	if( temp1 && temp2)
	{
		//handle TXE
		spi_TXE_interrupt_handle(pHandle);
	}

	//now check RXE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_RXNEIE);

	if( temp1 && temp2)
	{
		//handle TXE
		spi_RXNE_interrupt_handle(pHandle);
	}

	//check for ovr flag, (over run error)
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE);

	if( temp1 && temp2)
	{
		//handle RXNE
		spi_OVR_ERR_interrupt_handle(pHandle);
	}
}

/**
 * SPI_PeripheralControl
 *
 * function to HANDLE IRQ for spi
 *
 *
 * @param[in]: SPI_RegDef_t pSPIx : SPI periph register
 * @param[in]: enable/disable
 * @return : NONE
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);//enables periph
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);//disables periph
	}
}
/**
 * SPI_SSIConfig
 *
 * Function to config SSI
 *
 *
 * @param[in]: SPI_RegDef_t pSPIx : SPI periph register
 * @param[in]: enable/disable
 * @return : NONE
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);//enables periph
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);//disables periph
	}
}
/**
 * SPI_SSOEConfig
 *
 *Function to config SSOE
 *
 *
 * @param[in]: SPI_RegDef_t pSPIx : SPI periph register
 * @param[in]: enable/disable
 * @return : NONE
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);//enables periph
	}
	else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);//disables periph
	}
}



static void spi_OVR_ERR_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;

	//1. clear ovr flag
	if(pSPIHandle->TXState != SPI_BUSY_IN_TX)//not executed if SPI_TX busy and overrun error happens
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	//2. inform app
	SPI_ApplicationEventCallBack(pSPIHandle,SPI_EVENT_OVR_ERR);
}

static void spi_RXNE_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if(pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF))
	{
		//16 bit DFF
		//1. load data into DR
		*((uint16_t*)pSPIHandle->pRXBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RXLen -= 2;
		pSPIHandle->pRXBuffer -= 2;
	}
	else
	{
		//8 bit DFF
		//1. load data into DR
		*(pSPIHandle->pRXBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->RXLen--;
		pSPIHandle->pRXBuffer++;
	}

	if( ! pSPIHandle->RXLen)// if buffer length 0
	{
		//rxlen zero, reception of data is complete
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallBack(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

static void spi_TXE_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//2.  check the DFF bit in CR1
	if(pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF))
	{
		//16 bit DFF
		//1. load data into DR
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTXBuffer);
		pSPIHandle->TXLen -= 2;
		(uint16_t*)pSPIHandle->pTXBuffer++;
	}
	else
	{
		//8 bit DFF
		//1. load data into DR
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTXBuffer;
		pSPIHandle->TXLen--;
		pSPIHandle->pTXBuffer++;
	}

	if( ! pSPIHandle->TXLen)// if buffer length 0
	{
		//txlen zero, close spi comms, inform app that tx is over
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallBack(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)//if comms needs to end abruptly, this is called
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_TXEIE); //closing transmission code block
	pSPIHandle->pTXBuffer = NULL;
	pSPIHandle->TXLen = 0;
	pSPIHandle->TXState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)//if comms needs to end abruptly, this is called
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRXBuffer = NULL;
	pSPIHandle->RXLen = 0;
	pSPIHandle->RXState = SPI_READY;
}

void SPI_CLEAROVRFLAG(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;

}

__weak void SPI_ApplicationEventCallBack(SPI_Handle_t *pSPIHandle, uint8_t AppEvent)
{
	//weak implementation, application may over ride function
}
