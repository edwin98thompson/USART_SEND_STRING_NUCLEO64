/*
 * STM32_I2C_driver.c
 *
 *  Created on: 18 Sep 2020
 *      Author: edwin.thompson
 */

#include "STM32_I2C_driver.h"



static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);


static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= ( 1 << I2C_CR1_START);
}

static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1);	//SlaveAddr is slave address + read and write bit = 0
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummyRead = pI2CHandle->pI2Cx->SR1;
	dummyRead = pI2CHandle->pI2Cx->SR2;
	(void)dummyRead;

}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1; //SlaveAddr is Slave address + r/nw bit=1
	pI2Cx->DR = SlaveAddr;
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= ( 1 << I2C_CR1_STOP);
}

//periph clock setup
void I2C_PER_CLK_Control(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
			{
			if(pI2Cx == I2C1)
			{
				I2C1_PCLK_EN();
			}
			else if(pI2Cx == I2C2)
			{
				I2C2_PCLK_EN();
			}
			else if(pI2Cx == I2C3)
			{
				I2C3_PCLK_EN();
			}
		}
		else
		{
			if(pI2Cx == I2C1)
			{
				I2C1_PCLK_DI();
			}
			else if(pI2Cx == I2C2)
			{
				I2C2_PCLK_DI();
			}
			else if(pI2Cx == I2C3)
			{
				I2C3_PCLK_DI();
			}
		}
}


//init and deinit
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;

	//enable clockf for I2C periph
	I2C_PER_CLK_Control(pI2CHandle->pI2Cx, ENABLE);

	//1.ack control bit enables acknowledgement of transferred data
	tempreg |= pI2CHandle->I2C_Config.I2C_AckControl << 10;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//2. config freq field in the CR2 register
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() /1000000u;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	//3.config device address (when slave)
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR calculations
	//clock control register required to configure different clock control speeds
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//standard mode
		ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg |= ccr_value & 0xFFF;
	}
	else
	{
		//fast mode
		tempreg |= ( 1 << 15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDityCycle << 14);
		if(pI2CHandle->I2C_Config.I2C_FMDityCycle == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		else
		{
			ccr_value = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//TRISE config
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//standard mode
		tempreg = (RCC_GetPCLK1Value() /1000000u) + 1;
	}
	else
	{
		//fast mode
		tempreg = ((RCC_GetPCLK1Value() * 300 ) / 1000000000u ) + 1;
	}

	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3f);

}
void I2C_Deinit(I2C_RegDef_t *pI2Cx)
{
	//refer to RCC reset registers
}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTXBuffer, uint32_t Len, uint32_t SlaveAddr)
{
	//1. helper func to generate start cons
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm start generation complete by checking sb flag (start bit) in sr1
	// until sb cleared scl is stretched (pulled LOW)
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//3. send address of slave with r/nw bit set to w (0) (total 8 bits)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr);

	//4. confirm address phase complete by checking addr flag in SR1
	//while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//5. clear addr flag according to sw sequence
	//until addr cleared scl stretched
	I2C_ClearADDRFlag(pI2CHandle);

	//send data till len = 0
	while(Len > 0)
	{
		while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));//wait till txe is set
		pI2CHandle->pI2Cx->DR = *pTXBuffer;
		pTXBuffer++; //increase buffer
		Len--; //decrease length
	}

	//7. when Len becomes zero wait for TXE=1 and BTF=1 before generate stop condition
	// NOte: TXE = 1, BTF = 1 means both are empty, next transition shall begin
	// when BTF=1 SCL is stretched (pulled low)

	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));//wait till txe is set

	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));//wait till btf is set


	//8. Generate stop condition and master need not wait for completion of stop condition
	// Note: generating stop clears BTF
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRXBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr)
{

	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while( !  I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_SB)   );

	//3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits )
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,SlaveAddr);

	//4. wait until address phase is completed by checking the ADDR flag in teh SR1
	while( !  I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_ADDR)   );


	//procedure to read only 1 byte from slave
	if(Len == 1)
	{
		//Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);


		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//wait until  RXNE becomes 1
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE) );

		//generate STOP condition
		if(Sr == I2C_DISABLE_SR )
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//read data in to buffer
		*pRXBuffer = pI2CHandle->pI2Cx->DR;

	}


    //procedure to read data from slave when Len > 1
	if(Len > 1)
	{
		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//read the data until Len becomes zero
		for ( uint32_t i = Len ; i > 0 ; i--)
		{
			//wait until RXNE becomes 1
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE) );

			if(i == 2) //if last 2 bytes are remaining
			{
				//Disable Acking
				I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);

				//generate STOP condition
				if(Sr == I2C_DISABLE_SR )
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

			}

			//read the data from data register in to buffer
			*pRXBuffer = pI2CHandle->pI2Cx->DR;

			//increment the buffer address
			pRXBuffer++;

		}

	}

	//re-enable ACKing
	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_ENABLE);
	}

}

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == I2C_ACK_ENABLE)
	{
		//enable the ack
		pI2Cx->CR1 |= ( 1 << I2C_CR1_ACK);
	}else
	{
		//disable the ack
		pI2Cx->CR1 &= ~( 1 << I2C_CR1_ACK);
	}
}


//IRQ Configuration and ISR Handling
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

//Other peripheral control APIs
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}
	else
	{
		pI2Cx->CR1 &= ~(1 << 0);
	}
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


//application call back
void I2C_ApplicationEventCallBack(I2C_Handle_t *pI2CHandle, uint8_t AppEvent);
