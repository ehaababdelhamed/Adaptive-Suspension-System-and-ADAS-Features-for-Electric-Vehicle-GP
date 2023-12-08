/******************************************************************************
 * Module: Timer
 * File Name: Timer.c
 * Description: Source file for Timer driver for Stm32f103x6
 * Author: Ehab Mohamed Abdelhamed
 ******************************************************************************/

/*******************************************************************************
 *                             Includes			                            	*
 *******************************************************************************/
#include "stm32f103x6.h"
#include "Timer.h"
#include "GPIO.h"
/*******************************************************************************
 *								Global Variables	                   			*
 *******************************************************************************/
static void (*GP_callBackFunc[3])(void)={
		NULL,NULL,NULL
};

static void (*GP_ICUcallBackFunc[3][4])(void)={
		{NULL,NULL,NULL,NULL},
		{NULL,NULL,NULL,NULL},
		{NULL,NULL,NULL,NULL}
};

uint8 TIM_Channel_Port[3][4]={
		{GPIO_PORTA,GPIO_PORTA,GPIO_PORTA,GPIO_PORTA},
		{GPIO_PORTA,GPIO_PORTA,GPIO_PORTB,GPIO_PORTB},
		{GPIO_PORTB,GPIO_PORTB,GPIO_PORTB,GPIO_PORTB}
};

uint8 TIM_Channel_Pin[3][4]={
		{GPIO_PIN_0,GPIO_PIN_1,GPIO_PIN_2,GPIO_PIN_3},
		{GPIO_PIN_6,GPIO_PIN_7,GPIO_PIN_0,GPIO_PIN_1},
		{GPIO_PIN_6,GPIO_PIN_7,GPIO_PIN_8,GPIO_PIN_9}
};



/*******************************************************************************
 *                           		Macros	                   					*
 *******************************************************************************/

#define GET_TIM_BASE_ADD(TIMx_ID) 	(((TIMx_ID) == TIM2_ID)? (TIM2) : \
		((TIMx_ID) == TIM3_ID)? (TIM3) : \
				((TIMx_ID) == TIM4_ID)? (TIM4) : NULL)


/************************************************************************************
 * Function Name	: MCAL_Timer_init
 * Description		: Functional responsible for Initialize the Timer device by:
 * 1.Enable Timer Clock
 * 2. Select Timer Mode (UP - DOWN - UP/DOWN)
 * 3. Turn on/off reload register buffer
 * 4. Setup the Timer prescaler.
 * 5. Setup the value of Timer reload register.
 * Parameters (in)	: TimerConfig- pointer to Timer configuration parameters
 *  				  Timerx_ID - ID Number of Timer instance
 * Return value		: None
 * Note				: Stm32f103x6 Support only TIM2,TIM3 and TIM4
 ************************************************************************************/
void MCAL_GP_Timer_init(Timer_Instance_e Timerx_ID,GP_Timer_Config_t *TimerConfig){
	volatile GP_TIM_TypeDef * TIMx;		//to store the base address of TIM instance
	boolean error = FALSE;
	TIMx = GET_TIM_BASE_ADD(Timerx_ID);
	//Check if inputs is invalid
	if(TIMx == NULL ||TimerConfig == NULL ){
		error = TRUE;
	}

	if(error == FALSE){
		//1. Enable Timer Clock
		if(Timerx_ID == TIM2_ID){
			RCC_TIM2_CLK_EN();
			NVIC_IRQ28_TIM2_Enable();
		}else if(Timerx_ID == TIM3_ID){
			RCC_TIM3_CLK_EN();
			NVIC_IRQ29_TIM3_Enable();
		}else if(Timerx_ID == TIM4_ID){
			RCC_TIM4_CLK_EN();
			NVIC_IRQ30_TIM4_Enable();
		}

		//2. Select Timer Mode (UP - DOWN - UP/DOWN)
		if(TimerConfig->Timer_Counter_Mode == TIMER_UP_COUNTER || TimerConfig->Timer_Counter_Mode == TIMER_DOWN_COUNTER){
			TIMx->CR1.bit.DIR = TimerConfig->Timer_Counter_Mode;
		}else{
			TIMx->CR1.bit.CMS = TimerConfig->Timer_Counter_Mode;
		}

		//3. Setup the Timer prescaler.
		TIMx->PSC = TimerConfig->Prescaler_value;
	}
}


void MCAL_GP_Timer_Normal_Mode(Timer_Instance_e Timerx_ID,GP_Timer_NormalConfig_t *TimerNormalconfig){
	volatile GP_TIM_TypeDef * TIMx;		//to store the base address of TIM instance
	boolean error = FALSE;
	TIMx = GET_TIM_BASE_ADD(Timerx_ID);
	//Check if inputs is invalid
	if(TIMx == NULL ||TimerNormalconfig == NULL ){
		error = TRUE;
	}
	if(error == FALSE){
		//1. Turn on/off reload register buffer
		TIMx->CR1.bit.ARPE = TimerNormalconfig->Auto_Reload_Buffer_state;

		//2. Setup the value of Timer reload register.
		TIMx->ARR = TimerNormalconfig->Auto_reload_reg_value;
	}
}

void MCAL_GP_Timer_ICU_Mode(Timer_Instance_e Timerx_ID,GP_Timer_ICU_Config_t *TimerICUconfig){
	volatile GP_TIM_TypeDef * TIMx;		//to store the base address of TIM instance
	boolean error = FALSE;
	TIMx = GET_TIM_BASE_ADD(Timerx_ID);
	//Check if inputs is invalid
	if(TIMx == NULL ||TimerICUconfig == NULL ){
		error = TRUE;
	}

	//Select Trigger Source and Event
	TIMx->SMCR.bit.TS =TimerICUconfig->Source;
	TIMx->SMCR.bit.SMS=TimerICUconfig->Event;

	if(error == FALSE){
		switch(TimerICUconfig->Channel){
		case ICU_IC1_TI1:
		case ICU_IC1_TI2:
			//1. Select the direction of the channel (input/output) as well as the used input.
			TIMx->CCMR1.Input_bits.CC1S=TimerICUconfig->Channel;
			//2. defines the ratio of the prescaler acting.
			TIMx->CCMR1.Input_bits.IC1PSE=TimerICUconfig->Prescaler;
			//3. Select The input Trigger Edge
			TIMx->CCER.bit.CC1P = TimerICUconfig->edge;
			//4. Enable ICU Channel
			TIMx->CCER.bit.CC1E = TRUE;
			//6. Enable interrupt
			TIMx->DIER.bit.CC1IE = TRUE;
			GP_ICUcallBackFunc[Timerx_ID][0]=TimerICUconfig->CallBackFunc;
			break;
		case ICU_IC2_TI1:
		case ICU_IC2_TI2:
			//1. Select the direction of the channel (input/output) as well as the used input.
			TIMx->CCMR1.Input_bits.CC2S=(TimerICUconfig->Channel - 2);
			//2. defines the ratio of the prescaler acting.
			TIMx->CCMR1.Input_bits.IC2PSE=TimerICUconfig->Prescaler;
			//3. Select The input Trigger Edge
			TIMx->CCER.bit.CC2P = TimerICUconfig->edge;
			//4. Enable ICU Channel
			TIMx->CCER.bit.CC2E = TRUE;
			//6. Enable interrupt
			TIMx->DIER.bit.CC2IE = TRUE;
			GP_ICUcallBackFunc[Timerx_ID][1]=TimerICUconfig->CallBackFunc;
			break;
		case ICU_IC3_TI3:
		case ICU_IC3_TI4:
			//1. Select the direction of the channel (input/output) as well as the used input.
			TIMx->CCMR2.Input_bits.CC3S=(TimerICUconfig->Channel - 4);
			//2. defines the ratio of the prescaler acting.
			TIMx->CCMR2.Input_bits.IC3PSE=TimerICUconfig->Prescaler;
			//3. Select The input Trigger Edge
			TIMx->CCER.bit.CC3P = TimerICUconfig->edge;
			//4. Enable ICU Channel
			TIMx->CCER.bit.CC3E = TRUE;
			//6. Enable interrupt
			TIMx->DIER.bit.CC3IE = TRUE;
			GP_ICUcallBackFunc[Timerx_ID][2]=TimerICUconfig->CallBackFunc;
			break;
		case ICU_IC4_TI3:
		case ICU_IC4_TI4:
			//1. Select the direction of the channel (input/output) as well as the used input.
			TIMx->CCMR2.Input_bits.CC4S=(TimerICUconfig->Channel - 6);
			//2. defines the ratio of the prescaler acting.
			TIMx->CCMR2.Input_bits.IC4PSE=TimerICUconfig->Prescaler;
			//3. Select The input Trigger Edge
			TIMx->CCER.bit.CC4P = TimerICUconfig->edge;
			//4. Enable ICU Channel
			TIMx->CCER.bit.CC4E = TRUE;
			//6. Enable interrupt
			TIMx->DIER.bit.CC4IE = TRUE;
			GP_ICUcallBackFunc[Timerx_ID][3]=TimerICUconfig->CallBackFunc;
			break;
		}
		TIMx->ARR = 0xFFFF;
		//Enable Timer Counter
		TIMx->CR1.bit.CEN = TRUE;

		TIMx->CR2.bit.TI1S = TRUE;
	}
}


void MCAL_GP_Timer_ICU_setEdge(Timer_Instance_e Timerx_ID,GP_Timer_ICU_Channel_e Channel,GP_Timer_ICU_edge_e edge){
	volatile GP_TIM_TypeDef * TIMx;		//to store the base address of TIM instance
	boolean error = FALSE;
	TIMx = GET_TIM_BASE_ADD(Timerx_ID);
	//Check if inputs is invalid
	if(TIMx == NULL ){
		error = TRUE;
	}
	if(error == FALSE){
		switch(Channel){
		case ICU_IC1_TI1:
		case ICU_IC1_TI2:
			//Select The input Trigger Edge
			TIMx->CCER.bit.CC1P = edge;
			break;
		case ICU_IC2_TI1:
		case ICU_IC2_TI2:
			//Select The input Trigger Edge
			TIMx->CCER.bit.CC2P = edge;
			break;
		case ICU_IC3_TI3:
		case ICU_IC3_TI4:
			//Select The input Trigger Edge
			TIMx->CCER.bit.CC3P = edge;
			break;
		case ICU_IC4_TI3:
		case ICU_IC4_TI4:
			//Select The input Trigger Edge
			TIMx->CCER.bit.CC4P = edge;
			break;
		}
	}
}


uint16 MCAL_GP_Timer_ICU_GetValue(Timer_Instance_e Timerx_ID,GP_Timer_ICU_Channel_e Channel){
	volatile GP_TIM_TypeDef * TIMx;		//to store the base address of TIM instance
	boolean error = FALSE;
	uint16 CaptureValue;
	TIMx = GET_TIM_BASE_ADD(Timerx_ID);
	//Check if inputs is invalid
	if(TIMx == NULL ){
		error = TRUE;
	}
	if(error == FALSE){
		switch(Channel){
		case ICU_IC1_TI1:
		case ICU_IC1_TI2:
			CaptureValue=TIMx->CCR1;
			break;
		case ICU_IC2_TI1:
		case ICU_IC2_TI2:
			CaptureValue=TIMx->CCR2;
			break;
		case ICU_IC3_TI3:
		case ICU_IC3_TI4:
			CaptureValue=TIMx->CCR3;
			break;
		case ICU_IC4_TI3:
		case ICU_IC4_TI4:
			CaptureValue=TIMx->CCR4;
			break;
		}
	}
	return CaptureValue;
}




void MCAL_GP_Timer_ICU_Disable(Timer_Instance_e Timerx_ID,GP_Timer_ICU_Channel_e Channel){
	volatile GP_TIM_TypeDef * TIMx;		//to store the base address of TIM instance
	boolean error = FALSE;
	TIMx = GET_TIM_BASE_ADD(Timerx_ID);
	//Check if inputs is invalid
	if(TIMx == NULL ){
		error = TRUE;
	}
	if(error == FALSE){
		switch(Channel){
		case ICU_IC1_TI1:
		case ICU_IC1_TI2:
			//4. Enable ICU Channel
			TIMx->CCER.bit.CC1E = FALSE;
			//6. Enable interrupt
			TIMx->DIER.bit.CC1IE = FALSE;
			break;
		case ICU_IC2_TI1:
		case ICU_IC2_TI2:
			//4. Enable ICU Channel
			TIMx->CCER.bit.CC2E = FALSE;
			//6. Enable interrupt
			TIMx->DIER.bit.CC2IE = FALSE;
			break;
		case ICU_IC3_TI3:
		case ICU_IC3_TI4:
			//4. Enable ICU Channel
			TIMx->CCER.bit.CC3E = FALSE;
			//6. Enable interrupt
			TIMx->DIER.bit.CC3IE = FALSE;
			break;
		case ICU_IC4_TI3:
		case ICU_IC4_TI4:
			//4. Enable ICU Channel
			TIMx->CCER.bit.CC4E = FALSE;
			//6. Enable interrupt
			TIMx->DIER.bit.CC4IE = FALSE;
			break;
		}
	}
}


/************************************************************************************
 * Function Name	: MCAL_Timer_NormalModeStart
 * Description		: Function to Enable Timer counter and Timer Counter interrupt
 * Parameters (in)	: Timerx_ID - ID Number of Timer instance
 * Return value		: None
 ************************************************************************************/
void MCAL_Timer_Start(Timer_Instance_e Timerx_ID){
	volatile GP_TIM_TypeDef * TIMx;		//to store the base address of TIM instance
	boolean error = FALSE;
	TIMx = GET_TIM_BASE_ADD(Timerx_ID);
	//Check if inputs is invalid
	if(TIMx == NULL){
		error = TRUE;
	}
	if(error == FALSE){
		//Enable Timer interrupt
		TIMx->DIER.bit.UIE = TRUE;
		//Enable Timer Counter
		TIMx->CR1.bit.CEN = TRUE;
	}
}

/************************************************************************************
 * Function Name	: MCAL_GP_Timer_NormalModeStop
 * Description		: Function to Disable Timer counter and Timer Counter interrupt
 * Parameters (in)	: Timerx_ID - ID Number of Timer instance
 * Return value		: None
 ************************************************************************************/
void MCAL_GP_Timer_Stop(Timer_Instance_e Timerx_ID){
	volatile GP_TIM_TypeDef * TIMx;		//to store the base address of TIM instance
	boolean error = FALSE;
	TIMx = GET_TIM_BASE_ADD(Timerx_ID);
	//Check if inputs is invalid
	if(TIMx == NULL){
		error = TRUE;
	}
	if(error == FALSE){
		//Disable Timer interrupt
		TIMx->DIER.bit.UIE = FALSE;
		//Disable Timer Counter
		TIMx->CR1.bit.CEN = FALSE;
	}
}

/************************************************************************************
 * Function Name	: MCAL_Timer_SetCallBackFunc
 * Description		: Function to set call back function of Timer interrupt
 * Parameters (in)	: Timerx_ID - ID Number of Timer instance
 * 					  callBackFunc - Pointer to call back function
 * Return value		: None
 ************************************************************************************/
void MCAL_GP_Timer_SetCallBackFunc(Timer_Instance_e Timerx_ID,void (*callBackFunc)()){
	if(Timerx_ID <= TIM4_ID){
		//Set timer call back function
		GP_callBackFunc[Timerx_ID]=callBackFunc;
	}
}


void MCAL_GP_Timer_GPIO_Set_Pins(Timer_Instance_e Timerx_ID,GP_TIMER_MODES Mode,GP_Timer_Channel_e Channel){
	GPIO_PinConfig_t PinConfig;
	if(Mode == INPUT_CAPTURE_MODE){
		PinConfig=(GPIO_PinConfig_t){TIM_Channel_Pin[Timerx_ID][Channel],GPIO_INUPUT_FLOATING,GPIO_SPEED_INPUT};
		MCAL_GPIO_init(TIM_Channel_Port[Timerx_ID][Channel], &PinConfig);
	}else{
		PinConfig=(GPIO_PinConfig_t){TIM_Channel_Pin[Timerx_ID][Channel],GPIO_AF_OUTPUT_PP,GPIO_SPEED_10_MHZ};
		MCAL_GPIO_init(TIM_Channel_Port[Timerx_ID][Channel], &PinConfig);
	}

}

/*************************************** ISR *************************************/
void TIM2_IRQHandler(void){ 	/* TIM2 global interrupt*/
	if(TIM2->SR.bit.UIF == TRUE && GP_callBackFunc[TIM3_ID] != NULL){
		//Clear Update interrupt flag
		TIM2->SR.bit.UIF = FALSE;
		(*GP_callBackFunc[TIM2_ID])();
	}

	if(TIM2->SR.bit.CC1IF == TRUE && GP_ICUcallBackFunc[TIM2_ID][0] != NULL){
		TIM2->SR.bit.CC1IF = FALSE;
		(*GP_ICUcallBackFunc[TIM2_ID][0])();
	}
	if(TIM2->SR.bit.CC2IF == TRUE && GP_ICUcallBackFunc[TIM2_ID][1] != NULL){
		TIM2->SR.bit.CC2IF = FALSE;
		(*GP_ICUcallBackFunc[TIM2_ID][1])();
	}

	if(TIM2->SR.bit.CC3IF == TRUE && GP_ICUcallBackFunc[TIM2_ID][2] != NULL){
		TIM2->SR.bit.CC3IF = FALSE;
		(*GP_ICUcallBackFunc[TIM2_ID][2])();
	}

	if(TIM2->SR.bit.CC4IF == TRUE){
		TIM2->SR.bit.CC4IF = FALSE;
		(*GP_ICUcallBackFunc[TIM2_ID][3])();
	}
}


void TIM3_IRQHandler(void){ 	/* TIM3 global interrupt*/
	if(TIM3->SR.bit.UIF == TRUE || GP_callBackFunc[TIM3_ID] != NULL){
		//Clear Update interrupt flag
		TIM3->SR.bit.UIF = FALSE;
		(*GP_callBackFunc[TIM3_ID])();
	}

	if(TIM3->SR.bit.CC1IF == TRUE){
		TIM3->SR.bit.CC1IF = FALSE;
		(*GP_ICUcallBackFunc[TIM3_ID][0])();
	}
	if(TIM3->SR.bit.CC2IF == TRUE){
		TIM3->SR.bit.CC2IF = FALSE;
		(*GP_ICUcallBackFunc[TIM3_ID][1])();
	}

	if(TIM3->SR.bit.CC3IF == TRUE){
		TIM3->SR.bit.CC3IF = FALSE;
		(*GP_ICUcallBackFunc[TIM3_ID][2])();
	}

	if(TIM3->SR.bit.CC4IF == TRUE){
		TIM3->SR.bit.CC4IF = FALSE;
		(*GP_ICUcallBackFunc[TIM3_ID][3])();
	}
}

void TIM4_IRQHandler(void){ 	/* TIM3 global interrupt*/
	if(TIM4->SR.bit.UIF == TRUE ){
		//Clear Update interrupt flag
		TIM4->SR.bit.UIF = FALSE;
		(*GP_callBackFunc[TIM4_ID])();
	}

	if(TIM4->SR.bit.CC1IF == TRUE){
		TIM4->SR.bit.CC1IF = FALSE;
		(*GP_ICUcallBackFunc[TIM4_ID][0])();
	}
	if(TIM4->SR.bit.CC2IF == TRUE){
		TIM4->SR.bit.CC2IF = FALSE;
		(*GP_ICUcallBackFunc[TIM4_ID][1])();
	}

	if(TIM4->SR.bit.CC3IF == TRUE){
		TIM4->SR.bit.CC3IF = FALSE;
		(*GP_ICUcallBackFunc[TIM4_ID][2])();
	}

	if(TIM4->SR.bit.CC4IF == TRUE){
		TIM4->SR.bit.CC4IF = FALSE;
		(*GP_ICUcallBackFunc[TIM4_ID][3])();
	}
}
/*****************************************************************************/
