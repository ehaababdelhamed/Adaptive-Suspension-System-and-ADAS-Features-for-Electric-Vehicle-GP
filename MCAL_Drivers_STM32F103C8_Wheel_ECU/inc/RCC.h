 /******************************************************************************
 * Module: RCC
 * File Name: RCC.h
 * Description: Header file for Partial RCC driver for Stm32f103x6
 * Author: Ehab Mohamed Abdelhamed
 ******************************************************************************/

#ifndef INC_RCC_H_
#define INC_RCC_H_
#include "Platform_Types.h"

#define HSE_CLK (16000000UL)




#define HSI_CLK (8000000UL)
#define HSI (0U)
#define HSE (1U)
#define PLL (2U)

uint32 MCAL_RCC_GetPCLK2();
uint32 MCAL_RCC_GetPCLK1();
uint32 MCAL_RCC_GetHCLK();
uint32 MCAL_RCC_Get_SYSCLK();



#endif /* INC_RCC_H_ */
