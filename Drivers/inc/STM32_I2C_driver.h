/*
 * STM32_I2C_driver.h
 *
 *  Created on: 18 Sep 2020
 *      Author: edwin.thompson
 */

#ifndef INC_STM32_I2C_DRIVER_H_
#define INC_STM32_I2C_DRIVER_H_

#include "STM_NUCLEO_64.h"
#include "STM32_GPIO_driver.h"

//config structure

typedef struct
{
	uint32_t  I2C_SCLSpeed;
	uint8_t   I2C_DeviceAddress;
	uint8_t	  I2C_AckControl;
	uint8_t	  I2C_FMDityCycle;
}I2C_Config_t;

//I2C handle structure

typedef struct
{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
}I2C_Handle_t;

//I2C clk speeds
#define I2C_SCL_SPEED_SM	100000
#define I2C_SCL_SPEED_FM4K	400000
#define I2C_SCL_SPEED_FM2K	200000

//I2C ACK control
#define I2C_ACK_ENABLE		1
#define I2C_ACK_DISABLE		0

//I2C duty cycle FM
#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9	1

//implement for i2c
//I2C related Flags
#define I2C_FLAG_TXE					( 1 << I2C_SR1_TXE)
#define I2C_FLAG_RXNE					( 1 << I2C_SR1_RXNE)
#define I2C_FLAG_SB						( 1 << I2C_SR1_SB)
#define I2C_FLAG_OVR					( 1 << I2C_SR1_OVR)
#define I2C_FLAG_AF						( 1 << I2C_SR1_AF)
#define I2C_FLAG_ARLO					( 1 << I2C_SR1_ARLO)
#define I2C_FLAG_BERR					( 1 << I2C_SR1_BERR)
#define I2C_FLAG_STOPF					( 1 << I2C_SR1_STOPF)
#define I2C_FLAG_ADD10					( 1 << I2C_SR1_ADD10)
#define I2C_FLAG_BTF					( 1 << I2C_SR1_BTF)
#define I2C_FLAG_ADDR					( 1 << I2C_SR1_ADDR)
#define I2C_FLAG_TIMEOUT				( 1 << I2C_SR1_TIMEOUT)

#define I2C_DISABLE_SR  	RESET
#define I2C_ENABLE_SR   	SET


//SPI application states
#define SPI_READY						0
#define SPI_BUSY_IN_RX					1


/************************************************************************************************
 * 										APIs supported by this driver
 * 									Check func defs for more info on APIs
 ************************************************************************************************/

//periph clock setup
void I2C_PER_CLK_Control(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

//init and deinit
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_Deinit(I2C_RegDef_t *pI2Cx);
uint8_t RCC_GetPLLOutputCLK(void);
uint32_t RCC_GetPCLK1Value(void);

//data transmission
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTXBuffer, uint32_t Len, uint32_t SlaveAddr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRXBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr);
//IRQ Configuration and ISR Handling
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

//Other peripheral control APIs
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

//application call back
void I2C_ApplicationEventCallBack(I2C_Handle_t *pI2CHandle, uint8_t AppEvent);

#endif /* INC_STM32_I2C_DRIVER_H_ */
