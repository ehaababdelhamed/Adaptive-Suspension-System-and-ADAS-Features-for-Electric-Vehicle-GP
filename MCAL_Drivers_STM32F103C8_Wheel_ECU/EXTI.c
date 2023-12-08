/******************************************************************************
 * Module: EXTI
 * File Name: EXTI.c
 * Description: Source file for EXTI MCal driver for stm32f103x6
 * Author: Ehab Mohamed Abdelhamed
 ******************************************************************************/
#include <GPIO.h>
#include "EXTI.h"
#include "stm32f103x6.h"


/*******************************************************************************
 *                            Global variables	                   *
 *******************************************************************************/
//to store the call back function that set by call back function
void (*g_EXTI_callBackfunc[EXTI15])(void);

/*******************************************************************************
 *                            Private Function Prototypes	                   *
 *******************************************************************************/
static void EXTI_Update(EXTI_Config_t* EXTIConfig);

/*******************************************************************************
 *                      Function Definition                                    *
 *******************************************************************************/
/************************************************************************************
 * Function Name	: MCAL_EXTI_init
 * Description		: Function to initialize EXTI driver by Setup GPIO Pin,trigger edge
 * 					  ,map EXTI to GPIO pin
 * Parameters (in)	: EXTIConfig - Pointer to configuration data :EXTI Pin based on @REF EXTI_Defin
 * 					  and trigger edge
 * Return value		: None
 * Note				: Stm32f103x6 supported GPIOA,GPIOB,GPIOC,GPIOD,GPIOE
 * 					  but LQF48 Package supported fully GPIOA,GPIOB
 * 					  and partially GPIOC[13-15] and GPIOD[0-1]
 ************************************************************************************/
void MCAL_EXTI_init(EXTI_Config_t* EXTIConfig){
	EXTI_Update(EXTIConfig);
}
/************************************************************************************
 * Function Name	: MCAL_EXTI_deInit
 * Description		: Function to reset all EXTI registers
 * 					  Disable EXTI From NVIC
 * Parameters (in)	: None
 * Return value		: None
 * Note				: None
 ************************************************************************************/
void MCAL_EXTI_deInit(){
	//Assign Reset Value to all registers
	EXTI->EMR 	= 0x00000000 ;
	EXTI->IMR 	= 0x00000000 ;
	EXTI->FTSR 	= 0x00000000 ;
	EXTI->RTSR 	= 0x00000000 ;
	EXTI->SWIER = 0x00000000 ;
	EXTI->PR 	= 0xffffffff ;	//Write 1 to clear bit
	//Disable All EXTI from NVIC
	NVIC_IRQ6_EXTI0_Disable();
	NVIC_IRQ7_EXTI1_Disable();
	NVIC_IRQ8_EXTI2_Disable();
	NVIC_IRQ9_EXTI3_Disable();
	NVIC_IRQ10_EXTI4_Disable();
	NVIC_IRQ23_EXTI5_9_Disable();
	NVIC_IRQ40_EXTI10_15_Disable();
}
/************************************************************************************
 * Function Name	: MCAL_EXTI_enable
 * Description		: Function to enable Specific EXTI
 * Parameters (in)	: EXTI_Number -EXTI Number to be enabled
 * Return value		: None
 * Note				: The selected EXTI must be initialized before enable
 ************************************************************************************/
void MCAL_EXTI_enable(EXTI_NUM_e EXTI_Number){
	//Enable interrupt request for EXTIx
	if(EXTI_Number >= EXTI0 && EXTI_Number <= EXTI15){
		SET_BIT(EXTI->IMR , EXTI_Number);
	}
}
/************************************************************************************
 * Function Name	: MCAL_EXTI_disable
 * Description		: Function to disable Specific EXTI
 * Parameters (in)	: EXTI_Number -EXTI Number to be disabled
 * Return value		: None
 ************************************************************************************/
void MCAL_EXTI_disable(EXTI_NUM_e EXTI_Number){
	//Disable interrupt request for EXTIx
	if(EXTI_Number >= EXTI0 && EXTI_Number <= EXTI15){
	CLEAR_BIT(EXTI->IMR , EXTI_Number);
	}
}

/************************************************************************************
 * Function Name	: MCAL_EXTI_update
 * Description		: Function to Update EXTI driver by update GPIO Pin,trigger edge
 * 					  and mapping EXTI to GPIO pin.
 * Parameters (in)	: EXTIConfig - Pointer to configuration data :EXTI Pin based on @REF EXTI_Defin
 * 					  and trigger edge
 * Return value		: None
 * Note				: Stm32f103x6 supported GPIOA,GPIOB,GPIOC,GPIOD,GPIOE
 * 					  but LQF48 Package supported fully GPIOA,GPIOB
 * 					  and partially GPIOC[13-15] and GPIOD[0-1]
 ************************************************************************************/
void MCAL_EXTI_update(EXTI_Config_t* EXTIConfig){
	//Enable Rising Edge
	EXTI_Update(EXTIConfig);
}
/************************************************************************************
 * Function Name	: MCAL_EXTI_setCallBackFun
 * Description		: Function to setup call back function that be called  when interrupt occur
 * Parameters (in)	: EXTI_Number -EXTI Number to Set call back function for it
 * Return value		: None
 ************************************************************************************/
void MCAL_EXTI_setCallBackFun(EXTI_NUM_e EXTI_Number ,void (*callBackFunc)(void)){
	//set Call back function to global pointer to function
	if(callBackFunc != NULL || (EXTI_Number >= EXTI0 && EXTI_Number <= EXTI15) ){
		g_EXTI_callBackfunc[EXTI_Number]=callBackFunc;
	}
}

//=====================================================================================
/************************************************************************************
 * Function Name	: EXTI_Update
 * Description		: Private Function that configure EXTI driver
 * Parameters (in)	: EXTIConfig - Pointer to configuration data :EXTI Pin based on @REF EXTI_Defin
 * 					  and trigger edge
 * Return value		: None
 ************************************************************************************/
static void EXTI_Update(EXTI_Config_t* EXTIConfig){
	uint8 AFIO_EXTICR_Shift=0;
	uint8 AFIO_EXTICR_index=0;
	uint8 NVIC_Shift=0;
	//1-Configure GPIO to be AF input
	GPIO_PinConfig_t pinConfig={EXTIConfig->EXTI_Pin.GPIO_Pin,GPIO_AF_INPUT,GPIO_SPEED_INPUT};
	boolean error = FALSE;
	if(EXTIConfig == NULL){
		error = TRUE;
	}
	if( error == FALSE){
		MCAL_GPIO_init(EXTIConfig->EXTI_Pin.GPIO_Port,&pinConfig);
		//2-SeLect the register corresponding  to EXTI number
		//EXTICR1 [EXTI0 - EXTI3]
		//EXTICR2 [EXTI3 - EXTI7]
		//EXTICR3 [EXTI7 - EXTI11]
		//EXTICR4 [EXTI11 - EXTI5]
		AFIO_EXTICR_index = (uint8)(EXTIConfig->EXTI_Pin.EXTI_Num/4);
		AFIO_EXTICR_Shift = (EXTIConfig->EXTI_Pin.EXTI_Num % 4)*4;
		//3-Select the source input for EXTIx external interrupt (PA,PB,PC,PD)
		AFIO->EXTICR[AFIO_EXTICR_index] &= ~(uint32)(0xf << AFIO_EXTICR_Shift);		//Clear 4 bits
		AFIO->EXTICR[AFIO_EXTICR_index] |= ((EXTIConfig->EXTI_Pin.GPIO_Port & 0x0f) << AFIO_EXTICR_Shift);
		//3-Select the trigger condition (rising - failing - both)
		CLEAR_BIT(EXTI->RTSR,EXTIConfig->EXTI_Pin.EXTI_Num);
		CLEAR_BIT(EXTI->FTSR,EXTIConfig->EXTI_Pin.EXTI_Num);
		if(EXTIConfig->Edge == RAISING_EDGE ||EXTIConfig->Edge == BOTH_EDGES ){
			//Enable Rising Edge
			SET_BIT(EXTI->RTSR,EXTIConfig->EXTI_Pin.EXTI_Num);
		}
		if(EXTIConfig->Edge == FAILING_EGDE || EXTIConfig->Edge == BOTH_EDGES ){
			//Enable Falling Edge Edge
			SET_BIT(EXTI->FTSR,EXTIConfig->EXTI_Pin.EXTI_Num);
		}
		//4-Enable NVIC
		NVIC_Shift= (EXTIConfig->EXTI_Pin.IRQ_Num % 32);
		if(EXTIConfig->EXTI_Pin.EXTI_Num >= EXTI10){
			NVIC_ISER1 |= (1<<NVIC_Shift);
		}else{
			NVIC_ISER0 |= (1<<NVIC_Shift);
		}
	}
}

/*******************************************************************************
 *                     				ISR		                                   *
 *******************************************************************************/
void EXTI0_IRQHandler(){		/* EXTI Line0 interrupt*/
	SET_BIT(EXTI->PR,EXTI0);
	(*g_EXTI_callBackfunc[EXTI0])();
}
void EXTI1_IRQHandler(){		/* EXTI Line1 interrupt*/
	SET_BIT(EXTI->PR,EXTI1);
	(*g_EXTI_callBackfunc[EXTI1])();
}

void EXTI2_IRQHandler(){		/* EXTI Line2 interrupt*/
	SET_BIT(EXTI->PR,EXTI2);
	(*g_EXTI_callBackfunc[EXTI2])();
}
void EXTI3_IRQHandler(){		/* EXTI Line3 interrupt*/
	SET_BIT(EXTI->PR,EXTI3);
	(*g_EXTI_callBackfunc[EXTI3])();
}
void EXTI4_IRQHandler(){		/* EXTI Line4 interrupt*/
	SET_BIT(EXTI->PR,EXTI4);
	(*g_EXTI_callBackfunc[EXTI4])();
}
void EXTI9_5_IRQHandler(){		/* EXTI Line[9:5] interrupt*/
	if(BIT_IS_SET(EXTI->PR,EXTI5)){
		SET_BIT(EXTI->PR,EXTI5);
		(*g_EXTI_callBackfunc[EXTI5])();
	}
	if(BIT_IS_SET(EXTI->PR,EXTI6)){
		SET_BIT(EXTI->PR,EXTI6);
		(*g_EXTI_callBackfunc[EXTI6])();
	}
	if(BIT_IS_SET(EXTI->PR,EXTI7)){
		SET_BIT(EXTI->PR,EXTI7);
		(*g_EXTI_callBackfunc[EXTI7])();
	}
	if(BIT_IS_SET(EXTI->PR,EXTI8)){
		SET_BIT(EXTI->PR,EXTI8);
		(*g_EXTI_callBackfunc[EXTI8])();
	}
	if(BIT_IS_SET(EXTI->PR,EXTI9)){
		SET_BIT(EXTI->PR,EXTI9);
		(*g_EXTI_callBackfunc[EXTI9])();
	}
}

void EXTI15_10_IRQHandler(){  	/* EXTI Line[15:10] interrupts*/
	if(BIT_IS_SET(EXTI->PR,EXTI10)){
		SET_BIT(EXTI->PR,EXTI10);
		(*g_EXTI_callBackfunc[EXTI10])();
	}
	if(BIT_IS_SET(EXTI->PR,EXTI11)){
		SET_BIT(EXTI->PR,EXTI11);
		(*g_EXTI_callBackfunc[EXTI11])();
	}
	if(BIT_IS_SET(EXTI->PR,EXTI12)){
		SET_BIT(EXTI->PR,EXTI12);
		(*g_EXTI_callBackfunc[EXTI12])();
	}
	if(BIT_IS_SET(EXTI->PR,EXTI13)){
		SET_BIT(EXTI->PR,EXTI13);
		(*g_EXTI_callBackfunc[EXTI13])();
	}
	if(BIT_IS_SET(EXTI->PR,EXTI14)){
		SET_BIT(EXTI->PR,EXTI14);
		(*g_EXTI_callBackfunc[EXTI14])();
	}
	if(BIT_IS_SET(EXTI->PR,EXTI15)){
		SET_BIT(EXTI->PR,EXTI15);
		(*g_EXTI_callBackfunc[EXTI15])();
	}
}
//=========================================================================================
