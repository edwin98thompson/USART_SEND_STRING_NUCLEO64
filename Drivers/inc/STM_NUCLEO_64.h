/*
 *  STM_NUCLEO_64.h
 *
 *  Created on: Sep 10, 2020
 *      Author: edwin.thompson
 */

#ifndef INC_STM_NUCLEO_64_H_
#define INC_STM_NUCLEO_64_H_ //incase a header file is renamed, it may fuck things up, check this part

#include <stdint.h>
#include <stddef.h>

//#include "STM32_GPIO_driver.h"
//#include "STM32_SPI_driver.h"

#define __vo volatile
#define __weak __attribute__((weak))


/***************************start: Processor specific details *******************************************************
 *
 *  ARM Cortex M4 processor NVIC ISERx register addresses
 *  (Nested vectored interrupt controller (NVIC))
 */

#define NVIC_ISER0				( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1				( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2				( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3				( (__vo uint32_t*)0xE000E10C )

/*
 *ARM Cortex M4 processor NVIC ICERx register addresses
 */

#define NVIC_ICER0				( (__vo uint32_t*)0xE000E180 )
#define NVIC_ICER1				( (__vo uint32_t*)0xE000E184 )
#define NVIC_ICER2				( (__vo uint32_t*)0xE000E188 )
#define NVIC_ICER3				( (__vo uint32_t*)0xE000E18C )

#define NVIC_PR_BASE_ADDR		( (__vo uint32_t*)0xE000E400 )

#define NO_PR_BITS_IMPLEMENTED	4

//base addresses of ram and flash memories

#define FLASH_BASEADDR			0x08000000u 	//base address of flash memory
#define SRAM1_BASEADDR			0x20000000u 	//base address of system RAM1
#define SRAM2_BASEADDR			0x20001C00u 	//and 2
#define ROM_BASEADDR			0x1FFF0000u 	//system memory base address
#define SRAM 					SRAM1_BASEADDR 	//redefining

//base addresses of AHBX peripherals etc

#define PERIPH_BASE				0x40000000u
#define APB1PERIPH_BASE			PERIPH_BASE   	//APB advanced peripheral bus
#define APB2PERIPH_BASE			0x40010000u		// peripherals like uart, SPI, i2c and i/o
												//are subset of these base addresses
#define AHB1PERIPH_BASE			0x40020000u		//AHB advanced high-performance bus
#define AHB2PERIPH_BASE			0x50000000u

//base addresses of peripherals hanging off AHB1 bus

#define GPIOA_BASEADDR 			(AHB1PERIPH_BASE + 0x0000u)
#define GPIOB_BASEADDR 			(AHB1PERIPH_BASE + 0x0400u)
#define GPIOC_BASEADDR 			(AHB1PERIPH_BASE + 0x0800u)
#define GPIOD_BASEADDR 			(AHB1PERIPH_BASE + 0x0C00u)
#define GPIOE_BASEADDR 			(AHB1PERIPH_BASE + 0x1000u)
//#define GPIOF_BASEADDR 			(AHB1PERIPH_BASE + 0x1400u)
//#define GPIOG_BASEADDR 			(AHB1PERIPH_BASE + 0x1800u)
#define GPIOH_BASEADDR 			(AHB1PERIPH_BASE + 0x1C00u)
//#define GPIOI_BASEADDR 			(AHB1PERIPH_BASE + 0x2000u)
#define RCC_BASEADDR			(AHB1PERIPH_BASE + 0x3800)

//base addresses of peripherals hanging off APB1 bus

#define I2C1_BASEADDR 			(APB1PERIPH_BASE + 0x5400u)
#define I2C2_BASEADDR 			(APB1PERIPH_BASE + 0x5800u)
#define I2C3_BASEADDR 			(APB1PERIPH_BASE + 0x5C00u)
#define SPI2_BASEADDR 			(APB1PERIPH_BASE + 0x3800u)
#define SPI3_BASEADDR 			(APB1PERIPH_BASE + 0x3C00u)
#define USART2_BASEADDR 		(APB1PERIPH_BASE + 0x4400u)
//#define USART3_BASEADDR 		(APB1PERIPH_BASE + 0x4800u)

//base addresses of APB2 peripherals

#define EXTI_BASEADDR			(APB2PERIPH_BASE + 0x3C00u)// in embedded C programming
#define SPI1_BASEADDR			(APB2PERIPH_BASE + 0x3000u)// MACROS are upper
#define SPI4_BASEADDR			(APB2PERIPH_BASE + 0x3400u)
#define SYSCFG_BASEADDR			(APB2PERIPH_BASE + 0x3800u)
#define USART1_BASEADDR 		(APB2PERIPH_BASE + 0x1000u)
#define USART6_BASEADDR			(APB2PERIPH_BASE + 0x1400u)

//peripheral register definition struct for GPIO

typedef struct
{
	__vo uint32_t MODER;			//address offset: 0x00 GPIO port mode register
	__vo uint32_t OTYPER;			//address offset: 0x04 GPIO port output type register
	__vo uint32_t OSPEEDR;			//address offset: 0x08 GPIO port output speed register
	__vo uint32_t PUPDR;			//address offset: 0x0C GPIO port pull-up/pull-down register
	__vo uint32_t IDR;				//address offset: 0x10 GPIO port input data register
	__vo uint32_t ODR;				//address offset: 0x14 GPIO port output data register
	__vo uint32_t BSRR;				//address offset: 0x18 GPIO port bit set/reset register
	//__vo uint32_t BSRRH;				//address offset: 0x1A
	__vo uint32_t LCKR;				//address offset: 0x1C GPIO port configuration lock register
	__vo uint32_t AFR[2];			//address offset: 0x20-24 GPIO alternate function 0=lo 1=hi
}GPIO_RegDef_t;

//peripheral register definition for RCC

typedef struct
{
	__vo uint32_t CR;				//address offset: 0x00 RCC clock control register
	__vo uint32_t PLLCFGR;			//address offset: 0x04 RCC PLL configuration register
	__vo uint32_t CFGR;				//address offset: 0x08 RCC clock configuration register
	__vo uint32_t CIR;				//address offset: 0x0C RCC clock interrupt register
	__vo uint32_t AHB1RSTR;			//address offset: 0x10 AHB1 peripheral reset register
	__vo uint32_t AHB2RSTR;			//address offset: 0x14 RCC AHB2 peripheral reset register
	uint32_t RESERVED0[2];
	__vo uint32_t APB1RSTR;			//address offset: 0x20 RCC APB1 peripheral reset register
	__vo uint32_t APB2RSTR;			//address offset: 0x24 RCC APB2 peripheral reset register
	uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;	        //address offset: 0x30 RCC AHB1 peripheral clock enable register
	__vo uint32_t AHB2ENR;			//address offset: 0x34 RCC AHB2 peripheral clock enable register
	uint32_t RESERVED2[2];
	__vo uint32_t APB1ENR;			//address offset: 0x40 RCC APB1 peripheral clock enable register
	__vo uint32_t APB2ENR;			//address offset: 0x44 RCC APB2 peripheral clock enable register
	uint32_t RESERVED3[2];
	__vo uint32_t AHB1LPENR;		//address offset: 0x50 RCC AHB1 peripheral clock enable in low power mode register
	__vo uint32_t AHB2LPENR;		//address offset: 0x54 RCC AHB2 peripheral clock enable in low power mode register
	uint32_t RESERVED4[2];
	__vo uint32_t APB1LPENR;		//address offset: 0x60 RCC APB1 peripheral clock enable in low power mode register
	__vo uint32_t APB2LPENR;		//address offset: 0x64 RCC APB2 peripheral clock enabled in low power mode register
	uint32_t RESERVED5[2];
	__vo uint32_t BDCR;				//address offset: 0x70 RCC Backup domain control register
	__vo uint32_t CSR;				//address offset: 0x74 RCC clock control & status register
	uint32_t RESERVED6[2];
	__vo uint32_t SSCGR;			//address offset: 0x80 RCC spread spectrum clock generation register
	__vo uint32_t PLLI2SCFGR;		//address offset: 0x84 RCC PLLI2S configuration register
	uint32_t RESERVED7;
	__vo uint32_t DCKCFGR;			//address offset: 0x8C RCC Dedicated Clocks Configuration Register
}RCC_RegDef_t;

//peripheral register definition struct for EXTI

typedef struct
{
	__vo uint32_t IMR;				//address offset: 0x00 Interrupt mask register
	__vo uint32_t EMR;				//address offset: 0x04 Event mask register
	__vo uint32_t RTSR;				//address offset: 0x08 Rising trigger selection register
	__vo uint32_t FTSR;				//address offset: 0x0C Falling trigger selection register
	__vo uint32_t SWEIR;			//address offset: 0x10 Software interrupt event register
	__vo uint32_t PR;				//address offset: 0x14 Pending register
}EXTI_RegDef_t;

//peripheral register definition for syscfg stuff

typedef struct
{
	__vo uint32_t MEMRMP;			//address offset: 0x00 SYSCFG memory remap register
	__vo uint32_t PMC;				//address offset: 0x04 SYSCFG peripheral mode configuration register
	__vo uint32_t EXTICR[4];		//address offset: 0x08 SYSCFG external interrupt configuration register 1
	uint32_t	  RESERVED[2];		//address offset:
	__vo uint32_t CMPCR;			//address offset: 0x20 Compensation cell control register
	//uint32_t	  RESERVED[2];		//address offset:
}SYSCFG_RegDef_t;

//peripheral register definition for SPI stuff

typedef struct
{
	__vo uint32_t CR1;				//address offset: 0x00 SPI control register 1
	__vo uint32_t CR2;				//Address offset: 0x04 SPI control register 2
	__vo uint32_t SR;				//Address offset: 0x08 SPI status register
	__vo uint32_t DR;				//Address offset: 0x0C SPI data register
	__vo uint32_t CRCPR;			//Address offset: 0x10 SPI CRC polynomial register
	__vo uint32_t RXCRCR;			//Address offset: 0x14 SPI RX CRC register
	__vo uint32_t TXCRCR;			//Address offset: 0x18 SPI TX CRC register
	__vo uint32_t I2SCFGR;			//Address offset: 0x1C SPI_I2S configuration register
	__vo uint32_t I2SPR;			//Address offset: 0x20 SPI_I2S prescaler register
}SPI_RegDef_t;

//peripheral register definition for i2c stuff

typedef struct
{
	__vo uint32_t CR1;				//Address offset: 0x00 I2C Control register 1
	__vo uint32_t CR2;				//Address offset: 0x04 I2C Control register 2
	__vo uint32_t OAR1;				//Address offset: 0x08 I2C Own address register 1
	__vo uint32_t OAR2;				//Address offset: 0x0C I2C Own address register 2
	__vo uint32_t DR;				//Address offset: 0x10 I2C Data register
	__vo uint32_t SR1;				//Address offset: 0x14 I2C status register
	__vo uint32_t SR2;				//Address offset: 0x18 I2C status register 2
	__vo uint32_t CCR;				//Address offset: 0x1C I2C Clock control register
	__vo uint32_t TRISE;			//Address offset: 0x20 I2C TRISE register
	__vo uint32_t FLTR;				//Address offset: 0x24 I2C FLTR register

}I2C_RegDef_t;

typedef struct
{
	__vo uint32_t SR;				//Address offset: 0x00 Status register (USART_SR)
	__vo uint32_t DR;				//Address offset: 0x04 Data register (USART_DR)
	__vo uint32_t BRR;				//Address offset: 0x08 Baud rate register (USART_BRR)
	__vo uint32_t CR1;				//Address offset: 0x0c Control register 1 (USART_CR1)
	__vo uint32_t CR2;				//Address offset: 0x10 Control register 2 (USART_CR2)
	__vo uint32_t CR3;				//Address offset: 0x14 Control register 3 (USART_CR3)
	__vo uint32_t GTPR;				//Address offset: 0x18 Guard time and prescaler register (USART_GTPR)
}USART_RegDef_t;

//______________Peripheral_Definitions_________________________//

// GPIOA macro typecasted to base address of gpioA typecasted to GPIO register definitions

#define GPIOA					((GPIO_RegDef_t*)GPIOA_BASEADDR)// time saving
#define GPIOB					((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC					((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD					((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE					((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH					((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC						((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI					((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG					((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1 					((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2					((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3					((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4					((SPI_RegDef_t*)SPI4_BASEADDR)

#define I2C1					((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2					((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3					((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1					((USART_RegDef_t*)USART1_BASEADDR)
#define USART2					((USART_RegDef_t*)USART2_BASEADDR)
#define USART6					((USART_RegDef_t*)USART6_BASEADDR)

//clock enable macros for GPIO periphs

#define GPIOA_PCLK_EN()			( RCC->AHB1ENR |= (1 << 0) )//0th bit of AHB1ENR register to enable this port
#define GPIOB_PCLK_EN()			( RCC->AHB1ENR |= (1 << 1) )//etc
#define GPIOC_PCLK_EN()			( RCC->AHB1ENR |= (1 << 2) )
#define GPIOD_PCLK_EN()			( RCC->AHB1ENR |= (1 << 3) )
#define GPIOE_PCLK_EN()			( RCC->AHB1ENR |= (1 << 4) )
#define GPIOH_PCLK_EN()			( RCC->AHB1ENR |= (1 << 7) )

//clock enable macros for i2C periphs

#define I2C1_PCLK_EN()			( RCC->APB1ENR |= (1 << 21) )
#define I2C2_PCLK_EN()			( RCC->APB1ENR |= (1 << 22) )
#define I2C3_PCLK_EN()			( RCC->APB1ENR |= (1 << 23) )

//clock enable macros for SPI periphs

#define SPI1_PCLK_EN() 			( RCC->APB2ENR |= (1 << 12) )
#define SPI2_PCLK_EN() 			( RCC->APB1ENR |= (1 << 14) )
#define SPI3_PCLK_EN() 			( RCC->APB1ENR |= (1 << 15) )
#define SPI4_PCLK_EN()			( RCC->APB2ENR |= (1 << 13) )

//clock enable macros for USART periphs

#define USART1_PCLK_EN()		( RCC->APB2ENR |= (1 << 4)  )
#define USART2_PCLK_EN()		( RCC->APB1ENR |= (1 << 17) )
#define USART6_PCLK_EN()		( RCC->APB2ENR |= (1 << 5)  )

//clock enable macros for sys config

#define SYSCFG_PCLK_EN()		( RCC->APB2ENR |= (1 << 14) )

//clock disable macros for GPIO periphs

#define GPIOA_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 0) )//0th bit of AHB1ENR register to disable this port
#define GPIOB_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 1) )//etc
#define GPIOC_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 2) )
#define GPIOD_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 3) )
#define GPIOE_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 4) )
#define GPIOH_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 7) )

//clock disable macros for I2C periph

#define I2C1_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 21) )
#define I2C2_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 22) )
#define I2C3_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 23) )

//clock disable macros for SPI periphs

#define SPI1_PCLK_DI() 			( RCC->APB2ENR &= ~(1 << 12) )
#define SPI2_PCLK_DI() 			( RCC->APB1ENR &= ~(1 << 14) )
#define SPI3_PCLK_DI() 			( RCC->APB1ENR &= ~(1 << 15) )
#define SPI4_PCLK_DI() 			( RCC->APB2ENR &= ~(1 << 13) )

//clock disable macros for USART periphs

#define USART1_PCLK_DI()		( RCC->APB2ENR &= ~(1 << 4)  )
#define USART2_PCLK_DI()		( RCC->APB1ENR &= ~(1 << 17) )
#define USART6_PCLK_DI()		( RCC->APB2ENR &= ~(1 << 5)  )

//clock disable macros for sys config

#define SYSCFG_PCLK_DI()		( RCC->APB2ENR &= ~(1 << 14) )

//MACROS to reset GPIOx peripherals
//do while zero, technique in c to execute multiple statements using single macro

#define GPIOA_REG_RESET()		do{ ( RCC->AHB1RSTR |= (1 << 0) );	( RCC->AHB1RSTR &= ~(1 << 0) ); }while(0)
#define GPIOB_REG_RESET()		do{ ( RCC->AHB1RSTR |= (1 << 1) );	( RCC->AHB1RSTR &= ~(1 << 1) ); }while(0)
#define GPIOC_REG_RESET()		do{ ( RCC->AHB1RSTR |= (1 << 2) );	( RCC->AHB1RSTR &= ~(1 << 2) ); }while(0)
#define GPIOD_REG_RESET()		do{ ( RCC->AHB1RSTR |= (1 << 3) );	( RCC->AHB1RSTR &= ~(1 << 3) ); }while(0)
#define GPIOE_REG_RESET()		do{ ( RCC->AHB1RSTR |= (1 << 4) );	( RCC->AHB1RSTR &= ~(1 << 4) ); }while(0)
#define GPIOH_REG_RESET()		do{ ( RCC->AHB1RSTR |= (1 << 7) );	( RCC->AHB1RSTR &= ~(1 << 7) ); }while(0)

//returns port code for given GPIOx bae address

#define GPIO_BASE_ADDR_TO_CODE(x)  ((x == GPIOA) ? 0:\
									(x == GPIOB) ? 1:\
									(x == GPIOC) ? 2:\
									(x == GPIOD) ? 3:\
									(x == GPIOE) ? 4:\
									(x == GPIOH) ? 7:0 )

//IRQ external interrupt request numbers, specific to MCU

#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10		40


//SPI global interrupt request numbers
#define IRQ_SPI1				35
#define IRQ_SPI2				36
#define IRQ_SPI3				51
#define IRQ_SPI4				84

//some generic macros for use in funcs

#define ENABLE 					1
#define DISABLE 				0
#define SET 					ENABLE
#define RESET 					DISABLE
#define GPIO_PIN_SET 			SET
#define GPIO_PIN_RESET  		RESET
#define FLAG_RESET				RESET
#define FLAG_SET				SET

/**********************************************************************************
 * Bit position definitions of SPI peripherals
 **********************************************************************************/
// SPI_CR1
#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL			1
#define SPI_CR1_MSTR			2
#define SPI_CR1_BR				3
#define SPI_CR1_SPE				6
#define SPI_CR1_LSBFIRST		7
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RXONLY			10
#define SPI_CR1_DFF				11
#define SPI_CR1_CRCNEXT			12
#define SPI_CR1_CRCEN			13
#define SPI_CR1_BIDIOE			14
#define SPI_CR1_BIDIMODE		15

// SPI_CR2
#define SPI_CR2_RXDMAEN			0
#define SPI_CR2_TXDMAEN			1
#define SPI_CR2_SSOE			2
#define SPI_CR2_FRF				4
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXEIE			7

//SPI status registers
#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CHSIDE			2
#define SPI_SR_UDR				3
#define SPI_SR_CRCERR			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_BSY				7
#define SPI_SR_FRE				8

//bit position registers for I2C peripherals

//CR1 I2C register bits
#define I2C_CR1_PE				0
#define I2C_CR1_SMBUS			1
#define	I2C_CR1_SMBTYPE			3
#define I2C_CR1_ENARP			4
#define I2C_CR1_ENPAC			5
#define I2C_CR1_ENGC			6
#define I2C_CR1_NOSTRETCH		7
#define I2C_CR1_START			8
#define I2C_CR1_STOP			9
#define I2C_CR1_ACK				10
#define I2C_CR1_POS				11
#define I2C_CR1_PEC				12
#define I2C_CR1_ALERT			13
#define I2C_CR1_SWRST			15

//CR2 I2C reg bits
#define I2C_CR2_FREQ			0
#define I2C_CR2_ITERREN			8
#define I2C_CR2_ITEVTEN			9
#define I2C_CR2_ITBUFEN			10
#define I2C_CR2_DMAEN			11
#define I2C_CR2_LAST			12

//SR1 I2C reg bits
#define I2C_SR1_SB				0
#define I2C_SR1_ADDR			1
#define I2C_SR1_BTF				2
#define I2C_SR1_ADD10			3
#define I2C_SR1_STOPF			4
#define I2C_SR1_RXNE			6
#define I2C_SR1_TXE				7
#define I2C_SR1_BERR			8
#define I2C_SR1_ARLO			9
#define I2C_SR1_AF				10
#define I2C_SR1_OVR				11
#define I2C_SR1_PECERR			12
#define I2C_SR1_TIMEOUT			14
#define I2C_SR1_SMBALERT		15

//SR2 I2C reg bits
#define I2C_SR2_MSL				0
#define I2C_SR2_BUSY			1
#define I2C_SR2_TRA				2
#define I2C_SR2_GENCALL			4
#define I2C_SR2_SMBDEFAULT		5
#define I2C_SR2_SMBHOST			6
#define I2C_SR2_DUALF			7
#define I2C_SR2_PEC				8

//CCR I2C reg bits
#define I2C_CCR_CCR				0
#define I2C_CCR_DUTY			14
#define I2C_CCR_FS				15

//bit field definitions for usart peripherals

//status register bits
#define USART_SR_PE				0
#define USART_SR_FE				1
#define USART_SR_NF				2
#define USART_SR_ORE			3
#define USART_SR_IDLE			4
#define USART_SR_RXNE			5
#define USART_SR_TC				6
#define USART_SR_TXE			7
#define USART_SR_LBD			8
#define USART_SR_CTS			9

//data register bits
#define USART_DR_DR				0

//baud rate register bits
//#define USART_DR_DIV_FRACTION	0

/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
#define USART_CR1_OVER8  				15

/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14

/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11

#endif /* INC_STM_NUCLEO_64_H_ */
