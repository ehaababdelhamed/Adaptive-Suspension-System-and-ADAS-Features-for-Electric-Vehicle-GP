 /******************************************************************************
 * File Name: stm32f103x6.h
 * Description: MCU device header for stm32f103x6
 * Author: Ehab Mohamed Abdelhamed
 ******************************************************************************/

#ifndef INC_STM32F103X6_H_
#define INC_STM32F103X6_H_

/*******************************************************************************
 *                             		INCLUDES                            		*
 *******************************************************************************/
#include "Platform_Types.h"


/*******************************************************************************
 *						Base addresses for Memories                       		*
 *******************************************************************************/
#define FLASH_MEMORY_BASE							(0x08000000UL)
#define SYSTEM_MEMORY_BASE							(0x1FFFF000UL)
#define SRAM_MEMORY_BASE							(0x20000000UL)
#define PERIPHERALS_MEMORY_BASE						(0x40000000UL)
#define CORTEX_M3_INTERNAL_PERIPHERALS_MEMORY_BASE	(0xE0000000UL)

/*******************************************************************************
 *						Base addresses for Cortex M3 internal Peripherals                    	*
 *******************************************************************************/
//NVIC
#define NVIC_BASE		(0xE000E100UL)

/*******************************************************************************
 *						Base addresses for AHP Peripherals                    	*
 *******************************************************************************/
#define RCC_BASE		(0x40021000UL)


/*******************************************************************************
 *						Base addresses for APB1 Peripherals                    	*
 *******************************************************************************/
//USART
#define USART2_BASE 	(0x40004400UL)
#define USART3_BASE 	(0x40004800UL)

//SPI2
#define SPI2_BASE		(0x40003800UL)

//I2C
#define I2C1_BASE		(0x40005400UL)
#define I2C2_BASE		(0x40005800UL)

//TIM
#define TIM2_BASE		(0x40000000UL)
#define TIM3_BASE		(0x40000400UL)
#define TIM4_BASE		(0x40000800UL)


/*******************************************************************************
 *						Base addresses for APB2 Peripherals                    	*
 *******************************************************************************/

//GPIO
//LQF48 Package fully included PORTA and PORTB
#define GPIOA_BASE		(0x40010800UL)
#define GPIOB_BASE		(0x40010C00UL)

//LQF48 Package partially included PORTC and PORTD
#define GPIOC_BASE		(0x40011000UL)
#define GPIOD_BASE		(0x40011400UL)

//LQF48 Package not included PORTE
#define GPIOE_BASE		(0x40011800UL)

//EXTI
#define EXTI_BASE		(0x40010400UL)

//AFIO
#define AFIO_BASE		(0x40010000UL)

//USART1
#define USART1_BASE 	(0x40013800UL)

//SPI1
#define SPI1_BASE		(0x40013000UL)


//===============================================================================
/*******************************************************************************
 *							Peripherals Registers				               	*
 *******************************************************************************/
//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral register: GPIO
//-*-*-*-*-*-*-*-*-*-*-*
typedef struct{
	volatile uint32 CRL;
	volatile uint32 CRH;
	volatile uint32 IDR;
	volatile uint32 ODR;
	volatile uint32 BSRR;
	volatile uint32 BRR;
	volatile uint32 LCKR;
}GPIO_TypeDef;

//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral register: RCC
//-*-*-*-*-*-*-*-*-*-*-*

typedef struct{
	volatile uint32 CR;
	volatile uint32 CFGR;
	volatile uint32 CIR;
	volatile uint32 APB2RSTR;
	volatile uint32 APB1RSTR;
	volatile uint32 AHBENR;
	volatile uint32 APB2ENR;
	volatile uint32 APB1ENR;
	volatile uint32 BDCR;
	volatile uint32 CSR;
	volatile uint32 AHBSTR;
	volatile uint32 CFGR2;
}RCC_TypeDef;

//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral register: EXTI
//-*-*-*-*-*-*-*-*-*-*-*

typedef struct{
	volatile uint32 IMR;
	volatile uint32 EMR;
	volatile uint32 RTSR;
	volatile uint32 FTSR;
	volatile uint32 SWIER;
	volatile uint32 PR;
}EXTI_TypeDef;


//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral register: AFIO
//-*-*-*-*-*-*-*-*-*-*-*

typedef struct{
	volatile uint32 EVCR;
	volatile uint32 MAPR;
	volatile uint32 EXTICR[4];
	volatile uint32 RESERVED0;
	volatile uint32 MAPR2;
}AFIO_TypeDef;

//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral register: USART
//-*-*-*-*-*-*-*-*-*-*-*

/********************* Bit fields *************/
typedef union{
	volatile uint32 ALL_REG;
	struct{
		volatile uint32 PE:1;
		volatile uint32 FE:1;
		volatile uint32 NE:1;
		volatile uint32 ORE:1;
		volatile uint32 IDLE:1;
		volatile uint32 RXNE:1;
		volatile uint32 TC:1;
		volatile uint32 TXE:1;
		volatile uint32 LBD:1;
		volatile uint32 CTS:1;
	}bit;
}REG_USART_SR_t;

typedef union{
	volatile uint32 ALL_REG;
	struct{
		volatile uint32 DIV_Fraction:4;
		volatile uint32 DIV_Mantissa:12;
	}bit;
}REG_USART_BRR_t;

typedef union{
	volatile uint32 ALL_REG;
	struct{
		volatile uint32 SBK:1;
		volatile uint32 RWU:1;
		volatile uint32 RE_TE:2;
		volatile uint32 IDLEIE:1;
		volatile uint32 RXNEIE:1;
		volatile uint32 TCIE:1;
		volatile uint32 TXEIE:1;
		volatile uint32 PEIE:1;
		volatile uint32 PS_PCE:2;
		volatile uint32 WAKE:1;
		volatile uint32 M:1;
		volatile uint32 UE:1;
	}bit;
}REG_USART_CR1_t;

typedef union{
	volatile uint32 ALL_REG;
	struct{
		volatile uint32 ADD:4;
		volatile uint32 :1;
		volatile uint32 LBDL:1;
		volatile uint32 LBDIE:1;
		volatile uint32 :1;
		volatile uint32 LBCL:1;
		volatile uint32 CPHA:1;
		volatile uint32 CPOL:1;
		volatile uint32 CLKEN:1;
		volatile uint32 STOP:2;
		volatile uint32 LINEN:1;
	}bit;
}REG_USART_CR2_t;

typedef union{
	volatile uint32 ALL_REG;
	struct{
		volatile uint32 EIE:1;
		volatile uint32 IREN:1;
		volatile uint32 IRLP:1;
		volatile uint32 HDSEL:1;
		volatile uint32 NACK:1;
		volatile uint32 SCEN:1;
		volatile uint32 DMAR:1;
		volatile uint32 DMAT:1;
		volatile uint32 RTSE_CTSE:2;
		volatile uint32 CTSIE:1;
	}bit;
}REG_USART_CR3_t;
/********************* Reigsters *************/
typedef struct{
	volatile REG_USART_SR_t SR;
	volatile uint32 DR;
	volatile REG_USART_BRR_t BRR;
	volatile REG_USART_CR1_t CR1;
	volatile REG_USART_CR2_t CR2;
	volatile REG_USART_CR3_t CR3;
	volatile uint32 GTPR;
}USART_TypeDef;

//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral register: SPI
//-*-*-*-*-*-*-*-*-*-*-*
/********************* Bit fields *************/
typedef union{
	volatile uint32 ALL_REG;
	struct{
		volatile uint32 CPHA:1;
		volatile uint32 CPOL:1;
		volatile uint32 MSTR:1;
		volatile uint32 BR:3;
		volatile uint32 SPE:1;
		volatile uint32 LSBFIRST:1;
		volatile uint32 SSI_SSM:2;
		volatile uint32 RXONLY:1;
		volatile uint32 DFF:1;
		volatile uint32 CRCNEXT:1;
		volatile uint32 CRCEN:1;
		volatile uint32 BIDIOE:1;
		volatile uint32 BIDIMODE:1;
	}bit;
}REG_SPI_CR1_t;


typedef union{
	volatile uint32 ALL_REG;
	struct{
		volatile uint32 RXDMAEN:1;
		volatile uint32 TXDMAEN:1;
		volatile uint32 SSOE:1;
		volatile uint32 :2;
		volatile uint32 ERRIE:1;
		volatile uint32 RXNEIE:1;
		volatile uint32 TXEIE:1;
	}bit;
}REG_SPI_CR2_t;

typedef union{
	volatile uint32 ALL_REG;
	struct{
		volatile uint32 RXNE:1;
		volatile uint32 TXE:1;
		volatile uint32 CHSIDE:1;
		volatile uint32 UDR:1;
		volatile uint32 CRCERR:1;
		volatile uint32 MODF:1;
		volatile uint32 OVR:1;
		volatile uint32 BSY:1;
	}bit;
}REG_SPI_SR_t;


/********************* Registers ***************/
typedef struct{
	volatile REG_SPI_CR1_t CR1;
	volatile REG_SPI_CR2_t CR2;
	volatile REG_SPI_SR_t SR;
	volatile uint32 DR;
	volatile uint32 CRCPR;
	volatile uint32 RXCRCR;
	volatile uint32 TXCRCR;
	volatile uint32 I2SCFGR;
	volatile uint32 I2SPR;
}SPI_TypeDef;




//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral register: I2C
//-*-*-*-*-*-*-*-*-*-*-*

/************************* Bit Field *************************/
typedef union{
	volatile uint32 ALL_REG;
	struct{
		volatile uint32 PE:1;
		volatile uint32 SMBUS:1;
		volatile uint32 :1;
		volatile uint32 SMB_TYPE:1;
		volatile uint32 ENARP:1;
		volatile uint32 ENPEC:1;
		volatile uint32 ENGC:1;
		volatile uint32 NO_STRETCH:1;
		volatile uint32 START:1;
		volatile uint32 STOP:1;
		volatile uint32 ACK:1;
		volatile uint32 POS:1;
		volatile uint32 PEC:1;
		volatile uint32 ALERT:1;
		volatile uint32 :1;
		volatile uint32 SWRST:1;
	}bit;
}REG_I2C_CR1_t;


typedef union{
	volatile uint32 ALL_REG;
	struct{
		volatile uint32 FREQ:6;
		volatile uint32 :2;
		volatile uint32 ITERREN:1;
		volatile uint32 ITEVTEN:1;
		volatile uint32 ITBUFEN:1;
		volatile uint32 DMAEN:1;
		volatile uint32 LAST:1;
		volatile uint32 :3;
	}bit;
}REG_I2C_CR2_t;


typedef union{
	volatile uint32 ALL_REG;
	struct{
		volatile uint32 SB:1;
		volatile uint32 ADDR:1;
		volatile uint32 BTF:1;
		volatile uint32 ADD10:1;
		volatile uint32 STOPF:1;
		volatile uint32 :1;
		volatile uint32 RxNE:1;
		volatile uint32 TxE:1;
		volatile uint32 BERR:1;
		volatile uint32 ARLO:1;
		volatile uint32 AF:1;
		volatile uint32 OVR:1;
		volatile uint32 PEC_ERR:1;
		volatile uint32 :1;
		volatile uint32 TIME_OUT:1;
		volatile uint32 SMP_ALERT:1;
	}bit;
}REG_I2C_SR1_t;


typedef union{
	volatile uint32 ALL_REG;
	struct{
		volatile uint32 MSL:1;
		volatile uint32 BUSY:1;
		volatile uint32 TRA:1;
		volatile uint32 :1;
		volatile uint32 GEN_CALL:1;
		volatile uint32 SMBDE_FAULT:1;
		volatile uint32 SMB_HOST:1;
		volatile uint32 DUALF:1;
		volatile uint32 PEC:8;
	}bit;
}REG_I2C_SR2_t;

typedef union{
	volatile uint32 ALL_REG;
	struct{
		volatile uint32 CRR0_11:12;
		volatile uint32 :2;
		volatile uint32 DUTY:1;
		volatile uint32 F_S:1;
	}bit;
}REG_I2C_CRR_t;

typedef union{
	volatile uint32 ALL_REG;
	struct{
		volatile uint32 ADD:10;
		volatile uint32 :4;
		volatile uint32 BIT14:1;
		volatile uint32 ADD_MODE:1;
	}bit;
}REG_I2C_ORA1_t;

typedef union{
	volatile uint32 ALL_REG;
	struct{
		volatile uint32 ENDUAL:1;
		volatile uint32 ADD2:7;
	}bit;
}REG_I2C_ORA2_t;


/************************* Registers *************************/

typedef struct{
	volatile REG_I2C_CR1_t CR1;
	volatile REG_I2C_CR2_t CR2;
	volatile REG_I2C_ORA1_t OAR1;
	volatile REG_I2C_ORA2_t OAR2;
	volatile uint32 DR;
	volatile REG_I2C_SR1_t SR1;
	volatile REG_I2C_SR2_t SR2;
	volatile REG_I2C_CRR_t CCR;
	volatile uint32 TRISE;
}I2C_TypeDef;


//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral register: GP TIM2-TIM5
//-*-*-*-*-*-*-*-*-*-*-*


typedef union{
	volatile uint32 ALL_REG;
	struct{
		volatile uint32 CEN:1;
		volatile uint32 UDIS:1;
		volatile uint32 URS:1;
		volatile uint32 OPM:1;
		volatile uint32 DIR:1;
		volatile uint32 CMS:2;
		volatile uint32 ARPE:1;
		volatile uint32 CKD:2;
	}bit;
}REG_GP_TIM_CR1_t;

typedef union{
	volatile uint32 ALL_REG;
	struct{
		volatile uint32 :3;
		volatile uint32 CCDS:1;
		volatile uint32 MMS:3;
		volatile uint32 TI1S:1;
	}bit;
}REG_GP_TIM_CR2_t;

typedef union{
	volatile uint32 ALL_REG;
	struct{
		volatile uint32 SMS:3;
		volatile uint32 :1;
		volatile uint32 TS:3;
		volatile uint32 MSM:1;
		volatile uint32 ETF:4;
		volatile uint32 ETPS:2;
		volatile uint32 ECE:1;
		volatile uint32 ETP:1;
	}bit;
}REG_GP_TIM_SMCR_t;

typedef union{
	volatile uint32 ALL_REG;
	struct{
		volatile uint32 UIE:1;
		volatile uint32 CC1IE:1;
		volatile uint32 CC2IE:1;
		volatile uint32 CC3IE:1;
		volatile uint32 CC4IE:1;
		volatile uint32 Res:1;
		volatile uint32 TIE:1;
		volatile uint32 UDE:1;
		volatile uint32 CC1DE:1;
		volatile uint32 CC2DE:1;
		volatile uint32 CC3DE:1;
		volatile uint32 CC4DE:1;
		volatile uint32 TDE:1;
	}bit;
}REG_GP_TIM_DIER_t;


typedef union{
	volatile uint32 ALL_REG;
	struct{
		volatile uint32 UIF:1;
		volatile uint32 CC1IF:1;
		volatile uint32 CC2IF:1;
		volatile uint32 CC3IF:1;
		volatile uint32 CC4IF:1;
		volatile uint32 :1;
		volatile uint32 TIF:1;
		volatile uint32 :2;
		volatile uint32 CC1OF:1;
		volatile uint32 CC2OF:1;
		volatile uint32 CC3OF:1;
		volatile uint32 CC4OF:1;
	}bit;
}REG_GP_TIM_SR_t;

typedef union{
	volatile uint32 ALL_REG;
	struct{
		volatile uint32 UG:1;
		volatile uint32 CC1G:1;
		volatile uint32 CC2G:1;
		volatile uint32 CC3G:1;
		volatile uint32 CC4G:1;
		volatile uint32 Res:1;
		volatile uint32 TG:1;
	}bit;
}REG_GP_TIM_EGR_t;


typedef union{
	volatile uint32 ALL_REG;
	struct{
		volatile uint32 CC1S:2;
		volatile uint32 OC1FE:1;
		volatile uint32 OC1PE:1;
		volatile uint32 OC1M:3;
		volatile uint32 OC1CE:1;
		volatile uint32 CC2S:2;
		volatile uint32 OC2FE:1;
		volatile uint32 OC2PE:1;
		volatile uint32 OC2M:3;
		volatile uint32 OC2CE:1;
	}Output_bits;

	struct{
		volatile uint32 CC1S:2;
		volatile uint32 IC1PSE:2;
		volatile uint32 IC1F:4;
		volatile uint32 CC2S:2;
		volatile uint32 IC2PSE:2;
		volatile uint32 IC2F:4;
	}Input_bits;
}REG_GP_TIM_CCMR1_t;

typedef union{
	volatile uint32 ALL_REG;
	struct{
		volatile uint32 CC3S:2;
		volatile uint32 OC3FE:1;
		volatile uint32 OC3PE:1;
		volatile uint32 OC3M:3;
		volatile uint32 OC3CE:1;
		volatile uint32 CC4S:2;
		volatile uint32 OC4FE:1;
		volatile uint32 OC4PE:1;
		volatile uint32 OC4M:3;
		volatile uint32 OC4CE:1;
	}Output_bits;

	struct{
		volatile uint32 CC3S:2;
		volatile uint32 IC3PSE:2;
		volatile uint32 IC3F:4;
		volatile uint32 CC4S:2;
		volatile uint32 IC4PSE:2;
		volatile uint32 IC4F:4;
	}Input_bits;
}REG_GP_TIM_CCMR2_t;

typedef union{
	volatile uint32 ALL_REG;
	struct{
		volatile uint32 CC1E:1;
		volatile uint32 CC1P:1;
		volatile uint32 :2;
		volatile uint32 CC2E:1;
		volatile uint32 CC2P:1;
		volatile uint32 :2;
		volatile uint32 CC3E:1;
		volatile uint32 CC3P:1;
		volatile uint32 :2;
		volatile uint32 CC4E:1;
		volatile uint32 CC4P:1;
		volatile uint32 :2;


	}bit;
}REG_GP_TIM_CCER_t;


typedef struct{
	volatile REG_GP_TIM_CR1_t CR1;
	volatile REG_GP_TIM_CR2_t CR2;
	volatile REG_GP_TIM_SMCR_t SMCR;
	volatile REG_GP_TIM_DIER_t DIER;
	volatile REG_GP_TIM_SR_t SR;
	volatile REG_GP_TIM_EGR_t EGR;
	volatile REG_GP_TIM_CCMR1_t CCMR1;
	volatile REG_GP_TIM_CCMR2_t CCMR2;
	volatile REG_GP_TIM_CCER_t CCER;
	volatile uint32 CNT;
	volatile uint32 PSC;
	volatile uint32 ARR;
	volatile uint32 res1;
	volatile uint32 CCR1;
	volatile uint32 CCR2;
	volatile uint32 CCR3;
	volatile uint32 CCR4;
	volatile uint32 res2;
	volatile uint32 DCR;
	volatile uint32 DMAR;
}GP_TIM_TypeDef;

//======================================================================================
//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral Instants: GPIO
//-*-*-*-*-*-*-*-*-*-*-*
#define GPIOA		((GPIO_TypeDef*)(GPIOA_BASE))
#define GPIOB		((GPIO_TypeDef*)(GPIOB_BASE))
#define GPIOC		((GPIO_TypeDef*)(GPIOC_BASE))
#define GPIOD		((GPIO_TypeDef*)(GPIOD_BASE))
#define GPIOE		((GPIO_TypeDef*)(GPIOE_BASE))
//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral Instants: EXTI
//-*-*-*-*-*-*-*-*-*-*-*
#define EXTI		((EXTI_TypeDef*)(EXTI_BASE))

//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral Instants: AFIO
//-*-*-*-*-*-*-*-*-*-*-*
#define AFIO		((AFIO_TypeDef*)(AFIO_BASE))


//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral Instants: USART
//-*-*-*-*-*-*-*-*-*-*-*
#define USART1	 ((USART_TypeDef*)(USART1_BASE))
#define USART2	 ((USART_TypeDef*)(USART2_BASE))
#define USART3	 ((USART_TypeDef*)(USART3_BASE))

//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral Instants: SPI
//-*-*-*-*-*-*-*-*-*-*-*
#define SPI1	 ((SPI_TypeDef*)(SPI1_BASE))
#define SPI2	 ((SPI_TypeDef*)(SPI2_BASE))

//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral Instants: I2C
//-*-*-*-*-*-*-*-*-*-*-*
#define I2C1	 ((I2C_TypeDef*)(I2C1_BASE))
#define I2C2	 ((I2C_TypeDef*)(I2C2_BASE))

//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral Instants: I2C
//-*-*-*-*-*-*-*-*-*-*-*
#define TIM2	 ((GP_TIM_TypeDef*)(TIM2_BASE))
#define TIM3	 ((GP_TIM_TypeDef*)(TIM3_BASE))
#define TIM4	 ((GP_TIM_TypeDef*)(TIM4_BASE))

//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral Instants: RCC
//-*-*-*-*-*-*-*-*-*-*-*
#define RCC			((RCC_TypeDef*)(RCC_BASE))

//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral register: NVIC
//-*-*-*-*-*-*-*-*-*-*-*
//REG0 >> [IRQ0 - IRQ31]
//REG1 >> [IRQ32 - IRQ63]
//REG2 >> [IRQ64 - IRG67]
#define NVIC_ISER0	(*(volatile uint32*)(NVIC_BASE+0X00))
#define NVIC_ISER1	(*(volatile uint32*)(NVIC_BASE+0X04))
#define NVIC_ISER2	(*(volatile uint32*)(NVIC_BASE+0X08))
#define NVIC_ICER0	(*(volatile uint32*)(NVIC_BASE+0X80))
#define NVIC_ICER1	(*(volatile uint32*)(NVIC_BASE+0X84))
#define NVIC_ICER2	(*(volatile uint32*)(NVIC_BASE+0X88))


/*******************************************************************************
 *							Clock Enable Macros									*
 *******************************************************************************/
/*Macros for pins of RCC_APB2ENR*/
#define AFIOEN		(1U<<0)
#define IOPAEN		(1U<<2)
#define IOPBEN		(1U<<3)
#define IOPCEN		(1U<<4)
#define IOPDEN		(1U<<5)
#define IOPEEN		(1U<<6)

#define ADC1EN		(1U<<9)
#define ADC2EN		(1U<<10)
#define TIM1EN		(1U<<11)
#define SPI1EN		(1U<<12)
#define USART1EN	(1U<<14)
/*Macros for pins of RCC_APB1ENR*/
#define USART2EN	(1U<<17)
#define USART3EN	(1U<<18)
#define SPI2EN		(1U<<14)
#define I2C1EN		(1U<<21)
#define I2C2EN		(1U<<22)

#define TIM2EN		(1U<<0)
#define TIM3EN		(1U<<1)
#define TIM4EN		(1U<<2)


/*Enable Clock*/
/*APB2 Peripherals*/
#define RCC_AFIO_CLK_EN()		(RCC->APB2ENR|= AFIOEN)
#define RCC_GPIOA_CLK_EN()		(RCC->APB2ENR |= IOPAEN)
#define RCC_GPIOB_CLK_EN()		(RCC->APB2ENR |= IOPBEN)
#define RCC_GPIOC_CLK_EN()		(RCC->APB2ENR |= IOPCEN)
#define RCC_GPIOD_CLK_EN()		(RCC->APB2ENR |= IOPDEN)
#define RCC_GPIOE_CLK_EN()		(RCC->APB2ENR |= IOPEEN)
#define RCC_ADC1_CLK_EN()		(RCC->APB2ENR |= ADC1EN)
#define RCC_ADC2_CLK_EN()		(RCC->APB2ENR |= ADC2EN)
#define RCC_TIMER1_CLK_EN()		(RCC->APB2ENR |= TIM1EN)
#define RCC_SPI1_CLK_EN()		(RCC->APB2ENR |= SPI1EN)
#define RCC_USART1_CLK_EN()		(RCC->APB2ENR |= USART1EN)

/*APB1 Peripherals*/
#define RCC_USART2_CLK_EN()		(RCC->APB1ENR |= USART2EN)
#define RCC_USART3_CLK_EN()		(RCC->APB1ENR |= USART3EN)
#define RCC_SPI2_CLK_EN()		(RCC->APB1ENR |= SPI2EN)
#define RCC_I2C1_CLK_EN()		(RCC->APB1ENR |= I2C1EN)
#define RCC_I2C2_CLK_EN()		(RCC->APB1ENR |= I2C2EN)

#define RCC_TIM2_CLK_EN()		(RCC->APB1ENR |= TIM2EN)
#define RCC_TIM3_CLK_EN()		(RCC->APB1ENR |= TIM3EN)
#define RCC_TIM4_CLK_EN()		(RCC->APB1ENR |= TIM4EN)


/*Disable Clock*/



/*Reset Modules*/
/*APB2 Peripherals*/
#define RCC_AFIO_RESET()		(RCC->APB2RSTR |= AFIOEN)
#define RCC_GPIOA_RESET()		(RCC->APB2RSTR |= IOPAEN)
#define RCC_GPIOB_RESET()		(RCC->APB2RSTR |= IOPBEN)
#define RCC_GPIOC_RESET()		(RCC->APB2RSTR |= IOPCEN)
#define RCC_GPIOD_RESET()		(RCC->APB2RSTR |= IOPDEN)
#define RCC_GPIOE_RESET()		(RCC->APB2RSTR |= IOPEEN)

#define RCC_ADC1_RESET()		(RCC->APB2RSTR |= ADC1EN)
#define RCC_ADC2_RESET()		(RCC->APB2RSTR |= ADC2EN)
#define RCC_TIMER1_RESET()		(RCC->APB2RSTR |= TIM1EN)
#define RCC_SPI1_RESET()		(RCC->APB2RSTR |= SPI1EN)
#define RCC_USART1_RESET()		(RCC->APB2RSTR |= USART1EN)

/*APB1 Peripherals*/
#define RCC_USART2_RESET()		(RCC->APB1RSTR |= USART2EN)
#define RCC_USART3_RESET()		(RCC->APB1RSTR |= USART3EN)
#define RCC_SPI2_RESET()		(RCC->APB1RSTR |= SPI2EN)
#define RCC_I2C1_RESET()		(RCC->APB1RSTR |= I2C1EN)
#define RCC_I2C2_RESET()		(RCC->APB1RSTR |= I2C2EN)

#define RCC_TIM2_RESET()		(RCC->APB1RSTR |= TIM2EN)
#define RCC_TIM3_RESET()		(RCC->APB1RSTR |= TIM3EN)
#define RCC_TIM4_RESET()		(RCC->APB1RSTR |= TIM4EN)
/*******************************************************************************
 *							NCIC IRQ Enable/Disable Macros									*
 *******************************************************************************/
#define NVIC_IRQ6_EXTI0_Enable()		(NVIC_ISER0 |= 1<<6)
#define NVIC_IRQ7_EXTI1_Enable()		(NVIC_ISER0 |= 1<<7)
#define NVIC_IRQ8_EXTI2_Enable()		(NVIC_ISER0 |= 1<<8)
#define NVIC_IRQ9_EXTI3_Enable()		(NVIC_ISER0 |= 1<<9)
#define NVIC_IRQ10_EXTI4_Enable()		(NVIC_ISER0 |= 1<<10)
#define NVIC_IRQ23_EXTI5_9_Enable()		(NVIC_ISER0 |= 1<<23)
#define NVIC_IRQ40_EXTI10_15_Enable()	(NVIC_ISER1 |= 1<<8)	//40-32 = 8

#define NVIC_IRQ37_USART1_Enable()		(NVIC_ISER1 |= 1<<5)	//37-32 = 5
#define NVIC_IRQ38_USART2_Enable()		(NVIC_ISER1 |= 1<<6)	//38-32 = 6
#define NVIC_IRQ39_USART3_Enable()		(NVIC_ISER1 |= 1<<7)	//39-32 = 7

#define NVIC_IRQ35_SPI1_Enable()		(NVIC_ISER1 |= 1<<3)	//35-32 = 3
#define NVIC_IRQ36_SPI2_Enable()		(NVIC_ISER1 |= 1<<4)	//36-32 = 4

#define NVIC_IRQ31_I2C1_EV_Enable()		(NVIC_ISER0 |= 1<<31)	//31
#define NVIC_IRQ32_I2C1_ER_Enable()		(NVIC_ISER1 |= 1<<0)	//32-32 = 0

#define NVIC_IRQ33_I2C2_EV_Enable()		(NVIC_ISER1 |= 1<<1)	//33-32 = 1
#define NVIC_IRQ34_I2C2_ER_Enable()		(NVIC_ISER1 |= 1<<2)	//34-32 = 2

#define NVIC_IRQ28_TIM2_Enable()		(NVIC_ISER0 |= 1<<28)	//31
#define NVIC_IRQ29_TIM3_Enable()		(NVIC_ISER0 |= 1<<29)	//31
#define NVIC_IRQ30_TIM4_Enable()		(NVIC_ISER0 |= 1<<30)	//31


#define NVIC_IRQ6_EXTI0_Disable()		(NVIC_ICER0 |= 1<<6)
#define NVIC_IRQ7_EXTI1_Disable()		(NVIC_ICER0 |= 1<<7)
#define NVIC_IRQ8_EXTI2_Disable()		(NVIC_ICER0 |= 1<<8)
#define NVIC_IRQ9_EXTI3_Disable()		(NVIC_ICER0 |= 1<<9)
#define NVIC_IRQ10_EXTI4_Disable()		(NVIC_ICER0 |= 1<<10)
#define NVIC_IRQ23_EXTI5_9_Disable()	(NVIC_ICER0 |= 1<<23)
#define NVIC_IRQ40_EXTI10_15_Disable()	(NVIC_ICER1 |= 1<<8)	//40-32 = 8

#define NVIC_IRQ37_USART1_Disable()		(NVIC_ICER1 |= 1<<5)	//37-32 = 5
#define NVIC_IRQ38_USART2_Disable()		(NVIC_ICER1 |= 1<<6)	//38-32 = 6
#define NVIC_IRQ39_USART3_Disable()		(NVIC_ICER1 |= 1<<7)	//39-32 = 7

#define NVIC_IRQ35_SPI1_Disable()		(NVIC_ICER1 |= 1<<3)	//35-32 = 3
#define NVIC_IRQ36_SPI2_Disable()		(NVIC_ICER1 |= 1<<4)	//36-32 = 4

#define NVIC_IRQ31_I2C1_EV_Disable()	(NVIC_ICER0 |= 1<<31)	//31
#define NVIC_IRQ32_I2C1_ER_Disable()	(NVIC_ICER1 |= 1<<0)	//32-32 = 0
#define NVIC_IRQ33_I2C2_EV_Disable()	(NVIC_ICER1 |= 1<<1)	//33-32 = 1
#define NVIC_IRQ34_I2C2_ER_Disable()	(NVIC_ICER1 |= 1<<2)	//34-32 = 2

#define NVIC_IRQ28_TIM2_Disable()		(NVIC_ICER0 |= 1<<28)	//31
#define NVIC_IRQ29_TIM3_Disable()		(NVIC_ICER0 |= 1<<29)	//31
#define NVIC_IRQ30_TIM4_Disable()		(NVIC_ICER0 |= 1<<30)	//31


/*******************************************************************************
 *									IVT											*
 *******************************************************************************/
#define EXTI0_IRQ		(6U)
#define EXTI1_IRQ		(7U)
#define EXTI2_IRQ		(8U)
#define EXTI3_IRQ		(9U)
#define EXTI4_IRQ		(10U)
#define EXTI5_IRQ		(23U)
#define EXTI6_IRQ		(23U)
#define EXTI7_IRQ		(23U)
#define EXTI8_IRQ		(23U)
#define EXTI9_IRQ		(23U)
#define EXTI10_IRQ		(40U)
#define EXTI11_IRQ		(40U)
#define EXTI12_IRQ		(40U)
#define EXTI13_IRQ		(40U)
#define EXTI14_IRQ		(40U)
#define EXTI15_IRQ		(40U)

#define USART1_IRQ		(37U)
#define USART2_IRQ		(38U)
#define USART3_IRQ		(39U)

#define SPI1_IRQ		(35U)
#define SPI2_IRQ		(36U)

#define I2C1_EV_IRQ		(31U)
#define I2C1_ERR_IRQ	(32U)
#define I2C2_EV_IRQ		(33U)
#define I2C2_ERR_IRQ	(34U)

#define TIM2_IRQ		(28U)
#define TIM3_IRQ		(29U)
#define TIM4_IRQ		(30U)




#endif /* INC_STM32F103X6_H_ */
