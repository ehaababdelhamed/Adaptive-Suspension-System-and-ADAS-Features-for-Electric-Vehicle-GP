/******************************************************************************
 * Module: RCC
 * File Name: RCC.c
 * Description: Source file for Partial RCC driver for Stm32f103x6
 * Author: Ehab Mohamed Abdelhamed
 ******************************************************************************/
#include "RCC.h"
#include "stm32f103x6.h"

/*Set and cleared by software to control the division factor of the APB Low speed clock (PCLK1).
0xx: HCLK not divided
100: HCLK divided by 2
101: HCLK divided by 4
110: HCLK divided by 8
111: HCLK divided by 16
 */
uint32 APB_divisionFactors[8] = {0,0,0,0,1,2,3,4};

/*Bits 7:4 HPRE[3:0]: AHB prescaler
Set and cleared by software to control AHB clock division factor.
0xxx: SYSCLK not divided
1000: SYSCLK divided by 2
1001: SYSCLK divided by 4
1010: SYSCLK divided by 8
1011: SYSCLK divided by 16
1100: SYSCLK divided by 64
1101: SYSCLK divided by 128
1110: SYSCLK divided by 256
1111: SYSCLK divided by 512
 */
uint32 AHP_divisionFactors[16] = {0,0,0,0,0,0,0,0,1,2,3,4,6,7,8,9};

/*These bits are written by software to define the PLL multiplication factor. They can be written only
when PLL is disabled.
000x: Reserved
0010: PLL input clock x 4
0011: PLL input clock x 5
0100: PLL input clock x 6
0101: PLL input clock x 7
0110: PLL input clock x 8
0111: PLL input clock x 9
10xx: Reserved
1100: Reserved
1101: PLL input clock x 6.5
111x: Reserved*/

uint32 PLL_MulFactors[16] = {0,0,4,5,6,7,8,9};

uint32 GetPLLFreq();


uint32 MCAL_RCC_GetPCLK2(){
	//Bits 13:11 PPRE2[2:0]: APB high-speed prescaler (APB2)
	uint8 index= ((RCC->CFGR >> 11) & 0b111);
	return ((MCAL_RCC_GetHCLK()) >> APB_divisionFactors[index] );
}

uint32 MCAL_RCC_GetPCLK1(){
	//Bits 10:8 PPRE1[2:0]: APB Low-speed prescaler (APB1)
	uint8 index= ((RCC->CFGR >> 8) & 0b111);
	return ((MCAL_RCC_GetHCLK()) >> APB_divisionFactors[index] );
}

uint32 MCAL_RCC_GetHCLK(){
	//Bits 7:4 HPRE[3:0]: AHB prescaler
	uint8 index= ((RCC->CFGR >> 4) & 0b1111);
	return ((MCAL_RCC_Get_SYSCLK()) >> APB_divisionFactors[index] );
}

uint32 MCAL_RCC_Get_SYSCLK(){
	/*Bits 3:2 SWS[1:0]: System clock switch status
	Set and cleared by hardware to indicate which clock source is used as system clock.
	00: HSI oscillator used as system clock
	01: HSE oscillator used as system clock
	10: PLL used as system clock
	11: Not applicable
	 */
	uint8 ClockSource = ((RCC->CFGR >>2) & 0b11);
	uint32 CLK_Value=0;
	switch(ClockSource){
	case HSI:
		CLK_Value = HSI_CLK;
		break;
	case HSE:
		CLK_Value = HSE_CLK;
		break;
	case PLL:
		CLK_Value=GetPLLFreq();
		break;
	}
	return CLK_Value;
}


uint32 GetPLLFreq(){
	/*Bit 16 PLLSRC: PLL entry clock source
	0: HSI oscillator clock / 2 selected as PLL input clock
	1: Clock from PREDIV1 selected as PLL input clock*/
	uint32 PLLSource=0;
	uint8 index;
	if(RCC->CFGR & (1>>16)){
		/*Bit 17 PLLXTPRE: LSB of division factor PREDIV1
		 *  this bit controls if PREDIV1 divides its input clock by 2 (PLLXTPRE=1) or not (PLLXTPRE=0)
		 */
		PLLSource = HSE_CLK;
		if(RCC->CFGR & (1>>17)){
			PLLSource /=2 ;
		}
	}else{
		PLLSource= (HSI_CLK/2);
	}
	//Bits 21:18 PLLMUL[3:0]: PLL multiplication factor
	index = ((RCC->CFGR >> 18) & 0b1111);
	return (PLLSource*PLL_MulFactors[index]);
}
