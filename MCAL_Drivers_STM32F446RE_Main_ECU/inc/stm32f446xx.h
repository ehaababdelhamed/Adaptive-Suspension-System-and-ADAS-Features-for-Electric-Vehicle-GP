 /******************************************************************************
 * File Name: stm32f103x6.h
 * Description: MCU device header for stm32f466xx
 * Author: Ehab Mohamed Abdelhamed
 ******************************************************************************/

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

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
#define RCC_BASE		(0x40023800UL)



/*******************************************************************************
 *						Base addresses for APB1 Peripherals                    	*
 *******************************************************************************/

//
#define GPIOA_BASE		(0x40020000UL)
#define GPIOB_BASE		(0x40020400UL)
#define GPIOC_BASE		(0x40020800UL)
#define GPIOD_BASE		(0x40020C00UL)
#define GPIOE_BASE		(0x40021000UL)
#define GPIOF_BASE		(0x40021400UL)
#define GPIOG_BASE		(0x40021C00UL)
#define GPIOH_BASE		(0x40021800UL)


/*******************************************************************************
 *						Base addresses for APB2 Peripherals                    	*
 *******************************************************************************/





//===============================================================================
/*******************************************************************************
 *							Peripherals Registers				               	*
 *******************************************************************************/
//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral register:
//-*-*-*-*-*-*-*-*-*-*-*
typedef struct{
	volatile uint32 MODER;
	volatile uint32 OTYPER;
	volatile uint32 OSPEEDER;
	volatile uint32 PUPDR;
	volatile uint32 IDR;
	volatile uint32 ODR;
	volatile uint32 BSSR;
	volatile uint32 LCKR;
	volatile uint32 AFRL;
	volatile uint32 AFRH;
}GPIO_TypeDef;

//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral register: RCC
//-*-*-*-*-*-*-*-*-*-*-*

typedef struct{
	volatile uint32 CR;
	volatile uint32 PLL_CFGR;
	volatile uint32 CFGR;
	volatile uint32 CIR;
	volatile uint32 AHP1RSTR;
	volatile uint32 AHP2RSTR;
	volatile uint32 AHP3RSTR;
	volatile uint32 RES1;
	volatile uint32 APB1RSTR;
	volatile uint32 APB2RSTR;
	volatile uint32 RES2;
	volatile uint32 RES3;
	volatile uint32 AHP1ENR;
	volatile uint32 AHP2ENR;
	volatile uint32 AHP3ENR;
	volatile uint32 RES4;
	volatile uint32 APB1ENR;
	volatile uint32 APB2ENR;
	volatile uint32 RES5;
	volatile uint32 RES6;
	volatile uint32 AHP1LPENR;
	volatile uint32 AHP2LPENR;
	volatile uint32 AHP3LPENR;
	volatile uint32 RES7;
	volatile uint32 APB1LPENR;
	volatile uint32 APB2LPENR;
	volatile uint32 RES8;
	volatile uint32 RES9;
	volatile uint32 BDCR;
	volatile uint32 CSR;
	volatile uint32 RES10;
	volatile uint32 RES11;
	volatile uint32 SSCGR;
	volatile uint32 PLL2SCFGR;
	volatile uint32 PLL2SAICFGR;
	volatile uint32 CKGATENR;
	volatile uint32 DCKCFGR2;
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


//======================================================================================
//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral Instants:
//-*-*-*-*-*-*-*-*-*-*-*
#define GPIOA		((GPIO_TypeDef*)(GPIOA_BASE))
#define GPIOB		((GPIO_TypeDef*)(GPIOB_BASE))
#define GPIOC		((GPIO_TypeDef*)(GPIOC_BASE))
#define GPIOD		((GPIO_TypeDef*)(GPIOD_BASE))
#define GPIOE		((GPIO_TypeDef*)(GPIOE_BASE))
#define GPIOF		((GPIO_TypeDef*)(GPIOF_BASE))
#define GPIOG		((GPIO_TypeDef*)(GPIOG_BASE))
#define GPIOH		((GPIO_TypeDef*)(GPIOH_BASE))


//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral Instants: EXTI
//-*-*-*-*-*-*-*-*-*-*-*

#define EXTI		((EXTI_TypeDef*)(EXTI_BASE))
//-*-*-*-*-*-*-*-*-*-*-*-
//Peripheral Instants: AFIO
//-*-*-*-*-*-*-*-*-*-*-*

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
#define GPIOAEN		(1U<<0)
#define GPIOBEN		(1U<<1)
#define GPIOCEN		(1U<<2)
#define GPIODEN		(1U<<3)
#define GPIOEEN		(1U<<4)
#define GPIOFEN		(1U<<5)
#define GPIOGEN		(1U<<6)
#define GPIOHEN		(1U<<7)


/*Enable Clock*/
#define RCC_GPIOA_CLK_EN()		(RCC->AHP1ENR |= GPIOAEN)
#define RCC_GPIOB_CLK_EN()		(RCC->AHP1ENR |= GPIOBEN)
#define RCC_GPIOC_CLK_EN()		(RCC->AHP1ENR |= GPIOCEN)
#define RCC_GPIOD_CLK_EN()		(RCC->AHP1ENR |= GPIODEN)
#define RCC_GPIOE_CLK_EN()		(RCC->AHP1ENR |= GPIOEEN)
#define RCC_GPIOF_CLK_EN()		(RCC->AHP1ENR |= GPIOEEN)
#define RCC_GPIOG_CLK_EN()		(RCC->AHP1ENR |= GPIOEEN)
#define RCC_GPIOH_CLK_EN()		(RCC->AHP1ENR |= GPIOEEN)

/*Disable Clock*/



/*Reset Modules*/
#define RCC_GPIOA_RESET()		(RCC->AHP1RSTR |= GPIOAEN)
#define RCC_GPIOB_RESET()		(RCC->AHP1RSTR |= GPIOBEN)
#define RCC_GPIOC_RESET()		(RCC->AHP1RSTR |= GPIOCEN)
#define RCC_GPIOD_RESET()		(RCC->AHP1RSTR |= GPIODEN)
#define RCC_GPIOE_RESET()		(RCC->AHP1RSTR |= GPIOEEN)
#define RCC_GPIOF_RESET()		(RCC->AHP1RSTR |= GPIOFEN)
#define RCC_GPIOG_RESET()		(RCC->AHP1RSTR |= GPIOGEN)
#define RCC_GPIOH_RESET()		(RCC->AHP1RSTR |= GPIOHEN)



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


#define NVIC_IRQ6_EXTI0_Disable()		(NVIC_ICER0 |= 1<<6)
#define NVIC_IRQ7_EXTI1_Disable()		(NVIC_ICER0 |= 1<<7)
#define NVIC_IRQ8_EXTI2_Disable()		(NVIC_ICER0 |= 1<<8)
#define NVIC_IRQ9_EXTI3_Disable()		(NVIC_ICER0 |= 1<<9)
#define NVIC_IRQ10_EXTI4_Disable()		(NVIC_ICER0 |= 1<<10)
#define NVIC_IRQ23_EXTI5_9_Disable()	(NVIC_ICER0 |= 1<<23)
#define NVIC_IRQ40_EXTI10_15_Disable()	(NVIC_ICER1 |= 1<<8)	//40-32 = 8

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


#endif /* INC_STM32F446XX_H_ */
