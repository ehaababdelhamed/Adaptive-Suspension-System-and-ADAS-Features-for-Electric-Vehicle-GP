/******************************************************************************
 * Module: Timer
 * File Name: Timer.h
 * Description: Header file for Timer driver for Stm32f103x6
 * Author: Ehab Mohamed Abdelhamed
 ******************************************************************************/

#ifndef INC_GP_TIMER_H_
#define INC_GP_TIMER_H_


/*******************************************************************************
 *                             Includes			                            	*
 *******************************************************************************/
#include "Common_Macros.h"
#include "Platform_Types.h"

/*******************************************************************************
 *                             	Timer Data Types                             	*
 *******************************************************************************/
//@REF: Timer_Mode_e

typedef enum{
	NORMAL_MODE,
	INPUT_CAPTURE_MODE,
	OUTPUT_COMPARE,
	PWM_MODE
}GP_TIMER_MODES;

typedef enum{
	Timer_CH1,
	Timer_CH2,
	Timer_CH3,
	Timer_CH4
}GP_Timer_Channel_e;

typedef enum{
	TIMER_UP_COUNTER,
	TIMER_DOWN_COUNTER,
	TIMER_UP_DOWN_COUNTER_CLMODE1,
	TIMER_UP_DOWN_COUNTER_CLMODE2,
	TIMER_UP_DOWN_COUNTER_CLMODE3
}GP_Timer_Counter_Modes;

/* CR1 >> Bit 4 DIR: Direction.
 * 0: Counter used as up counter.
 * 1: Counter used as down counter.
 * */

/*
CR1 >> Bits 6:5 CMS: Center-aligned mode selection
00: Edge-aligned mode. The counter counts up or down depending on the direction bit (DIR).
01: Center-aligned mode 1. The counter counts up and down alternatively. Output compare
interrupt flags of channels configured in output (CCxS=00 in TIMx_CCMRx register) are set
only when the counter is counting down.
10: Center-aligned mode 2. The counter counts up and down alternatively. Output compare
interrupt flags of channels configured in output (CCxS=00 in TIMx_CCMRx register) are set
only when the counter is counting up.
11: Center-aligned mode 3. The counter counts up and down alternatively. Output compare
interrupt flags of channels configured in output (CCxS=00 in TIMx_CCMRx register) are set
both when the counter is counting up or down.
*/

//@REF: Timer_AutoReloadPreload_state_e
/*
Bit 7 ARPE: Auto-reload preload enable
0: TIMx_ARR register is not buffered
1: TIMx_ARR register is buffered
*/

typedef enum{
	Timer_Auto_ReloadPreload_Buffer_Disable,
	Timer_Auto_ReloadPreload_Buffer_Enable,
}GP_Timer_ARR_buffer_state_e;


typedef struct{
	GP_Timer_Counter_Modes Timer_Counter_Mode;					//Specifies Timer Mode [UP - DOWN -UP/DOWN]
																//This parameter can be a value of @REF: Timer_Mode
	uint16 Prescaler_value;										//Specifies PRESCALER of Timer [0 - 65535]
	uint16 Counter_intial_value;								//Specifies initial value of timer counter [0-65535]
}GP_Timer_Config_t;


typedef struct{
	uint16 Auto_reload_reg_value;								//Specifies reload_reg_value (Compare match value)[0-65535]
	GP_Timer_ARR_buffer_state_e Auto_Reload_Buffer_state; 		//specifies Auto Reload Preload state [enable buffeing of reload value]																//This parameter can be a value of @REF: Timer_AutoReloadPreload_state_e
}GP_Timer_NormalConfig_t;

/*Bits 1:0 CC1S: Capture/Compare 1 selection
This bit-field defines the direction of the channel (input/output) as well as the used input.
00: CC1 channel is configured as output
01: CC1 channel is configured as input, IC1 is mapped on TI1
10: CC1 channel is configured as input, IC1 is mapped on TI2
11: CC1 channel is configured as input, IC1 is mapped on TRC. This mode is working only if
an internal trigger input is selected through TS bit (TIMx_SMCR register)
Note: CC1S bits are writable only when the channel is OFF (CC1E = 0 in TIMx_CCER).*/
typedef enum{
	ICU_IC1_TI1=1,
	ICU_IC1_TI2,
	ICU_IC2_TI2,
	ICU_IC2_TI1,
	ICU_IC3_TI3,
	ICU_IC3_TI4,
	ICU_IC4_TI4,
	ICU_IC4_TI3
}GP_Timer_ICU_Channel_e;

typedef enum{
	ICU_Rising_edge,
	ICU_Faling_edge
}GP_Timer_ICU_edge_e;

typedef enum{
	ICU_No_Prescaler,
	ICU_Prescaler_2,
	ICU_Prescaler_4,
	ICU_Prescaler_8
}GP_Timer_ICU_prescaler_e;

//101: Filtered Timer Input 1 (TI1FP1)
//110: Filtered Timer Input 2 (TI2FP2)
typedef enum{
	ICU_CH1_TI1FP1=5,
	ICU_CH2_TI2FP2
}GP_Timer_ICU_TriggerSource_e;

/*/
100: Reset Mode - Rising edge of the selected trigger input (TRGI) reinitializes the counter
and generates an update of the registers.
101: Gated Mode - The counter clock is enabled when the trigger input (TRGI) is high. The
counter stops (but is not reset) as soon as the trigger becomes low. Both start and stop of
the counter are controlled.
110: Trigger Mode - The counter starts at a rising edge of the trigger TRGI (but it is not reset).
Only the start of the counter is controlled.
 */

typedef enum{
	ICU_Reset_Event=4,
	ICU_Gated_Event,
	ICU_Trigger_Event
}GP_Timer_ICU_TriggerEvent_e;


typedef struct{
	GP_Timer_ICU_Channel_e Channel;
	GP_Timer_ICU_edge_e edge;						//Specifies Timer Mode [UP - DOWN -UP/DOWN]
																//This parameter can be a value of @REF: Timer_Mode
	GP_Timer_ICU_prescaler_e Prescaler;							//Specifies PRESCALER of Timer [0 - 65535]
	GP_Timer_ICU_TriggerSource_e Source;
	GP_Timer_ICU_TriggerEvent_e Event;
	void (*CallBackFunc)();
}GP_Timer_ICU_Config_t;

typedef enum{
	TIM2_ID,
	TIM3_ID,
	TIM4_ID
}Timer_Instance_e;


/*******************************************************************************
 *                             	Timer Definition                             	*
 *******************************************************************************/

/*******************************************************************************
 *					APIs Supported by "MCAL Timer DRIVER" 	                   *
 *******************************************************************************/

/************************************************************************************
 * Function Name	: MCAL_Timer_init
 * Description		: Functional responsible for Initialize the Timer device by:
 * 1.Enable Timer Clock
 * 2.Enable UART
 * 3.Setup the Frame format like number of data bits, parity bit type and number of stop bits.
 * 4.Setup the Timer baud rate.
 * 5.Enable/Disable hardware flow control RTS/CTS
 * 6.Enable TX/RX
 * Parameters (in)	: TimerConfig- pointer to Timer configuration parameters
 * 					  [baud rate - Number of data bits -Parity type - Number of stop bits]
 *  				  Timerx_ID - ID Number of UART instance
 * Return value		: None
 * Note				: Stm32f103x6 Support only UART1 and UART2
 ************************************************************************************/
void MCAL_GP_Timer_init(Timer_Instance_e Timerx_ID,GP_Timer_Config_t *TimerConfig);
void MCAL_GP_Timer_Normal_Mode(Timer_Instance_e Timerx_ID,GP_Timer_NormalConfig_t *TimerNormalconfig);
void MCAL_GP_Timer_ICU_Mode(Timer_Instance_e Timerx_ID,GP_Timer_ICU_Config_t *TimerICUconfig);
void MCAL_GP_Timer_ICU_setEdge(Timer_Instance_e Timerx_ID,GP_Timer_ICU_Channel_e Channel,GP_Timer_ICU_edge_e edge);
uint16 MCAL_GP_Timer_ICU_GetValue(Timer_Instance_e Timerx_ID,GP_Timer_ICU_Channel_e Channel);
void MCAL_GP_Timer_ICU_Disable(Timer_Instance_e Timerx_ID,GP_Timer_ICU_Channel_e Channel);
void MCAL_Timer_Start(Timer_Instance_e Timerx_ID);
void MCAL_GP_Timer_Stop(Timer_Instance_e Timerx_ID);
void MCAL_GP_Timer_SetCallBackFunc(Timer_Instance_e Timerx_ID,void (*callBackFunc)());
void MCAL_GP_Timer_GPIO_Set_Pins(Timer_Instance_e Timerx_ID,GP_TIMER_MODES Mode,GP_Timer_Channel_e Channel);




#endif /* INC_GP_TIMER_H_ */
