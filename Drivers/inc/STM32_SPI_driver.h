/*
 * STM32_SPI_driver.h
 *
 *  Created on: 15 Sep 2020
 *      Author: edwin.thompson
 */

#ifndef INC_STM32_SPI_DRIVER_H_

#define INC_STM32_SPI_DRIVER_H_

#include "STM_NUCLEO_64.h"
#include "STM32_GPIO_driver.h"

//Config structure for SPIx Peripheral

typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;

//Handle structure for SPIx peripheral

typedef struct
{
	SPI_RegDef_t	*pSPIx;
	SPI_Config_t	SPIConfig;
	uint8_t 		*pTXBuffer;
	uint8_t			*pRXBuffer;
	uint32_t		TXLen;
	uint32_t		RXLen;
	uint32_t		TXState;
	uint32_t		RXState;
}SPI_Handle_t;

//SPI device modes
#define SPI_DEVICE_MODE_SLAVE			0
#define SPI_DEVICE_MODE_MASTER			1


//spi bus config
#define SPI_BUS_CONFIG_FDUPLEX			1
#define SPI_BUS_CONFIG_HDUPLEX			2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3


//spi clock speeds
#define SPI_SCLK_SPEED_DIV2				0
#define SPI_SCLK_SPEED_DIV4				1
#define SPI_SCLK_SPEED_DIV8				2
#define SPI_SCLK_SPEED_DIV16			3
#define SPI_SCLK_SPEED_DIV32			4
#define SPI_SCLK_SPEED_DIV64			5
#define SPI_SCLK_SPEED_DIV128			6
#define SPI_SCLK_SPEED_DIV256			7

//spi dff modes
#define SPI_DFF_8BITS 					0
#define SPI_DFF_16BITS 					1

//CPOL
#define SPI_CPOL_LOW					0		//clock 0 when idle
#define SPI_CPOL_HIGH					1		//clock 1 when idle

//CPHA
#define SPI_CPHA_LOW					0		//1st transition 1st data capture edge
#define SPI_CPHA_HIGH					1		//2nd transition 1st data capture edge


//SPI SSM
#define SPI_SSM_DI						0
#define SPI_SSM_EN						1

//SPI related Flags
#define SPI_TXE_FLAG					( 1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG					( 1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG					( 1 << SPI_SR_BSY)

//SPI application states
#define SPI_READY						0
#define SPI_BUSY_IN_RX					1
#define SPI_BUSY_IN_TX					2

//possible SPI application events
#define SPI_EVENT_TX_CMPLT				1
#define SPI_EVENT_RX_CMPLT				2
#define SPI_EVENT_OVR_ERR				3
#define SPI_EVENT_CRC_ERR				4


/************************************************************************************************
 * 										APIs supported by this driver
 * 									Check func defs for more info on APIs
 ************************************************************************************************/

//periph clock setup
void SPI_PER_CLK_Control(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

//init and deinit
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_Deinit(SPI_RegDef_t *pSPIx);

//Data send and receive
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTXBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRXBuffer, uint32_t Len);

//Data send and receive (interrupt mode)
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTXBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRXBuffer, uint32_t Len);

//IRQ Configuration and ISR Handling
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

//Other peripheral control APIs
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_CLEAROVRFLAG(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

//application call back
void SPI_ApplicationEventCallBack(SPI_Handle_t *pSPIHandle, uint8_t AppEvent);


#endif /* INC_STM32_SPI_DRIVER_H_ */
