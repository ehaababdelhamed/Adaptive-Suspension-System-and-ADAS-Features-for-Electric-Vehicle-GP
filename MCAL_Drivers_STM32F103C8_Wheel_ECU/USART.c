/******************************************************************************
 * Module: USART
 * File Name: USART.c
 * Description: Source file for USART driver for Stm32f103x6
 * Author: Ehab Mohamed Abdelhamed
 ******************************************************************************/

/*******************************************************************************
 *                             Includes			                            	*
 *******************************************************************************/
#include "stm32f103x6.h"
#include "SPI.h"
#include "USART.h"
#include "RCC.h"
#include "GPIO.h"

/*******************************************************************************
 *                            			Global Variables	                   				*
 *******************************************************************************/
static void (*GP__callBackFunc[NUM_OF_UART_IRQ])(void)={NULL,NULL,NULL};
static USART_TypeDef *USART_instance[NUM_OF_UART_INSTANCES]= {USART1,USART2,USART3};
static USART_Config_t Global_USART_Config[NUM_OF_UART_INSTANCES];

/* USART1 :				USART2 :			USART3 :
 * PA9  >> TX			PA2 >> TX			PB10  >> TX
 * PA10 >> RX			PA3 >> RX			PB11 >> RX
 * PA11 >> CTS			PA0 >> CTS			PB13 >> CTS
 * PA12 >> RTS			PA1 >> RTS			PB14 >> RTS
 * */
//USART GPIO PORT
static uint8 USARTxPORT[NUM_OF_UART_INSTANCES]={
		GPIO_PORTA,
		GPIO_PORTA,
		GPIO_PORTB
};
//USART GPIO Pins
//{ TX_PIN , RX,PIN , CTS_PIN , RTS_PIN}
static uint8 USARTxPINS[NUM_OF_UART_INSTANCES][4]={
		{GPIO_PIN_9,GPIO_PIN_10,GPIO_PIN_11,GPIO_PIN_12},
		{GPIO_PIN_2,GPIO_PIN_3,GPIO_PIN_0,GPIO_PIN_1},
		{GPIO_PIN_10,GPIO_PIN_11,GPIO_PIN_13,GPIO_PIN_14}
};

/*******************************************************************************
 *                            	Function Definition	                   			*
 *******************************************************************************/
/************************************************************************************
 * Function Name	: MCAL_USART_init
 * Description		: Functional responsible for Initialize the USART device by:
 * 1.Enable UART Clock
 * 2.Enable UART
 * 3.Setup the Frame format like number of data bits, parity bit type and number of stop bits.
 * 4.Setup the USART baud rate.
 * 5.Enable/Disable hardware flow control RTS/CTS
 * 6.Enable TX/RX
 * Parameters (in)	: USARTConfig- pointer to USART configuration parameters
 * 					  [baud rate - Number of data bits -Parity type - Number of stop bits]
 *  				  USARTx_ID - ID Number of UART instance
 * Return value		: None
 * Note				: Stm32f103x6 Support only USART1, USART2 and USART3
 ************************************************************************************/
void MCAL_USART_init(USART_Instance_e USARTx_ID,USART_Config_t *USARTConfig){
	uint32 Fclk=0;				//to store The APBx clock
	uint16 Mantissa;			//to store the integer value of UBRR
	uint8 Friction;				//to store the friction value of UBRR
	uint32 Mantissa_Mul100;
	uint32 Div_Mul100;
	USART_TypeDef * USARTx;		//to store the base address of USART instance
	boolean error = FALSE;
	USARTx = USART_instance[USARTx_ID];
	//Check if inputs is invalid
	if(USARTx == NULL ||USARTConfig == NULL ){
		error = TRUE;
	}
	if(error == FALSE){
		Global_USART_Config[USARTx_ID] = (USART_Config_t)(*USARTConfig);
		/***** 1.Enable UART Clock *****/
		if(USARTx_ID == USART1_ID){
			RCC_USART1_CLK_EN();
			Fclk =MCAL_RCC_GetPCLK2();
		}else if(USARTx_ID == USART2_ID){
			RCC_USART2_CLK_EN();
			Fclk =MCAL_RCC_GetPCLK1();
		}else if(USARTx_ID == USART3_ID){
			RCC_USART3_CLK_EN();
			Fclk =MCAL_RCC_GetPCLK1();
		}
		/***** 2.Enable UART *****/
		USARTx->CR1.bit.UE = TRUE;
		/***3.Setup the Frame format >> number of data bits, parity bit type and number of stop bits.***/
		USARTx->CR1.bit.M = USARTConfig->Data_Length;
		USARTx->CR1.bit.PS_PCE = USARTConfig->Parity;
		USARTx->CR2.bit.STOP = USARTConfig->Stop_Bit;
		/***** 4.Setup the USART baud rate *****/
		/*
		 * USARTDIV = Fclk / (16 * Baudrate)
		 * USARTDIV_MUL100 = uint32((100 *Fclk ) / (16 * Baudrate) == (25 *Fclk ) / (4* Baudrate) )
		 * DIV_Mantissa_MUL100 = Integer Part (USARTDIV ) * 100
		 * DIV_Mantissa = Integer Part (USARTDIV )
		 * DIV_Fraction = (( USARTDIV_MUL100 - DIV_Mantissa_MUL100 ) * 16 ) / 100
		 * */
		Mantissa = (uint16)(Fclk/(16*USARTConfig->Boud_Rate));
		Div_Mul100 = ((25*Fclk)/(4*USARTConfig->Boud_Rate));
		Mantissa_Mul100 = Mantissa*100;
		Friction = (16*(Div_Mul100 - Mantissa_Mul100))/100;
		USARTx->BRR.bit.DIV_Mantissa = Mantissa;
		USARTx->BRR.bit.DIV_Fraction = Friction;

		/***** 5.Enable TX/RX *****/
		USARTx->CR1.bit.RE_TE = USARTConfig->USART_Mode;
		/***** 6.Enable/Disable hardware flow control RTS/CTS *****/
		USARTx->CR3.bit.RTSE_CTSE = USARTConfig->HwFlowCtrl;

	}
}

/************************************************************************************
 * Function Name	: MCAL_USART_deInit
 * Description		: Functional responsible for reset the all USART register.
 * 					  Disable USART.
 * Parameters (in)	: USARTx_ID - ID Number of UART instance
 * Return value		: None
 * Note				: None
 ************************************************************************************/
void MCAL_USART_deInit(USART_Instance_e USARTx_ID){
	USART_TypeDef * USARTx;
	boolean error = FALSE;
	USARTx = USART_instance[USARTx_ID];
	//Check if inputs is invalid
	if(USARTx == NULL ){
		error = TRUE;
	}
	if(error == FALSE){
		/*Reset USART*/
		if(USARTx_ID == USART1_ID){
			RCC_USART1_RESET();
		}else if(USARTx_ID == USART2_ID){
			RCC_USART2_RESET();
		}else if(USARTx_ID == USART3_ID){
			RCC_USART3_RESET();
		}
	}
}
/************************************************************************************
 * Function Name	: MCAL_USART_setCallBackFunc
 * Description		: Functional responsible Enable USART interrupt and set call back Function.
 * 					  Disable USART.
 * Parameters (in)	: USARTx_ID - ID Number of UART instance
 * 					  IRQx - IRQ Type depend on @REF: USART_IRQ_Types
 * 					  callBackFunc - Function that is called when interrupt is triggered
 * Return value		: None
 * Note				: None
 ************************************************************************************/
void MCAL_USART_setCallBackFunc(USART_Instance_e USARTx_ID,USART_IRQ_e IRQx,void (*callBackFunc)()){
	USART_TypeDef * USARTx;
	boolean error = FALSE;
	USARTx = USART_instance[USARTx_ID];
	if(USARTx == NULL || callBackFunc == NULL){
		error = TRUE;
	}
	if(error == FALSE){
		//Enable NVIC IRQ
		if(USARTx_ID == USART1_ID){
			NVIC_IRQ37_USART1_Enable();
		}else if(USARTx_ID == USART2_ID){
			NVIC_IRQ38_USART2_Enable();
		}else if(USARTx_ID == USART3_ID){
			NVIC_IRQ39_USART3_Enable();
		}
		//Enable UART IRQ
		SET_BIT(USARTx->CR1.ALL_REG,IRQx);
		GP__callBackFunc[(IRQx-IRQ_SHIFT)] = callBackFunc;
	}
}
/************************************************************************************
 * Function Name	: MCAL_USART_sendData
 * Description		: Functional responsible to send data by USART.
 * Parameters (in)	: USARTx_ID - ID Number of UART instance
 *  				  TXD_Buffer : data to be send by USART
 *  				  Pooling - Select [USART_POOLING_ENABLE - USART_POOLING_DISABLE]
 * Return value		: None
 * Note				: None
 ************************************************************************************/
void MCAL_USART_sendData(USART_Instance_e USARTx_ID,uint16 TXD_Buffer,USART_PoolingMechanism_e Pooling){
	USART_TypeDef * USARTx;
	boolean error = FALSE;
	USARTx = USART_instance[USARTx_ID];
	if(USARTx == NULL ){
		error = TRUE;
	}
	if(error == FALSE){
		if(Pooling == USART_POOLING_ENABLE){
			//Wait until TX buffer is empty
			while(USARTx->SR.bit.TXE != TRUE);
		}
		/* When transmitting with the parity enabled (PCE bit set to 1 in the USART_CR1 register),
		 * the value written in the MSB (bit 7 or bit 8 depending on the data length) has no effect
		 * because it is replaced by the parity.
		 * When receiving with the parity enabled, the value read in the MSB bit is the received parity bit.
		 * */
		if(Global_USART_Config[USARTx_ID].Data_Length == USART_DATE_lENGTH_8_BITS){
			USARTx->DR = (TXD_Buffer & 0x00ff);
		}else if(Global_USART_Config[USARTx_ID].Data_Length == USART_DATE_lENGTH_9_BITS){
			USARTx->DR = (TXD_Buffer & 0x01ff);
		}

	}
}
/************************************************************************************
 * Function Name	: MCAL_USART_receiveData
 * Description		: Functional responsible to receive data by USART.
 * Parameters (in)	: USARTx_ID - ID Number of UART instance
 *  				  Pooling - Select [USART_POOLING_ENABLE - USART_POOLING_DISABLE]
 * Return value		: Data to be received
 * Note				: None
 ************************************************************************************/
uint16 MCAL_USART_receiveData(USART_Instance_e USARTx_ID,USART_PoolingMechanism_e Pooling){
	USART_TypeDef * USARTx;
	boolean error = FALSE;
	uint16 RXDbuffer=0;
	USARTx = USART_instance[USARTx_ID];
	if(USARTx == NULL ){
		error = TRUE;
	}
	if(error == FALSE){
		if(Pooling == USART_POOLING_ENABLE){
			while(USARTx->SR.bit.RXNE != TRUE);
		}
		if(Global_USART_Config[USARTx_ID].Parity == USART_DISAPLE_PARITY){
			if(Global_USART_Config[USARTx_ID].Data_Length == USART_DATE_lENGTH_8_BITS){
				RXDbuffer = (USARTx->DR & 0x00ff);
			}else if(Global_USART_Config[USARTx_ID].Data_Length == USART_DATE_lENGTH_9_BITS){
				RXDbuffer = (USARTx->DR & 0x01ff);
			}
		}else{
			//the value written in the MSB (bit 7 or bit 8 depending on the data length) has no effect
			//because it is replaced by the parity
			if(Global_USART_Config[USARTx_ID].Data_Length == USART_DATE_lENGTH_8_BITS){
				RXDbuffer = (USARTx->DR & 0x007f);
			}else if(Global_USART_Config[USARTx_ID].Data_Length == USART_DATE_lENGTH_9_BITS){
				RXDbuffer = (USARTx->DR & 0x00ff);
			}
		}
	}
	return RXDbuffer;
}

/************************************************************************************
 * Function Name	: MCAL_USART_WAIT_TC
 * Description		: Functional responsible to wait until TC flag is set;
 * Parameters (in)	: USARTx_ID - ID Number of UART instance
 * Return value		: Data to be received
 * Note				: None
 ************************************************************************************/
void MCAL_USART_WAIT_TC(USART_Instance_e USARTx_ID){
	USART_TypeDef * USARTx;
	boolean error = FALSE;
	USARTx = USART_instance[USARTx_ID];
	if(USARTx == NULL ){
		error = TRUE;
	}
	if(error == FALSE){
		while(USARTx->SR.bit.TC != TRUE);
	}
}

/************************************************************************************
 * Function Name	: MCAL_USART_GPIO_SET_PINS
 * Description		: Functional responsible to set GPIO pins of USART.
 * Parameters (in)	: USARTx_ID - ID Number of UART instance
 * Return value		: Character to be received
 * Note				: None
 ************************************************************************************/
void MCAL_USART_GPIO_SET_PINS(USART_Instance_e USARTx_ID){
	USART_TypeDef * USARTx;
	boolean error = FALSE;
	GPIO_PinConfig_t Pin_Config;
	USARTx = USART_instance[USARTx_ID];
	if(USARTx == NULL ){
		error = TRUE;
	}
	if(error == FALSE){
		//Configure TX Pin
		Pin_Config = (GPIO_PinConfig_t){USARTxPINS[USARTx_ID][0],GPIO_AF_OUTPUT_PP,GPIO_SPEED_2_MHZ};
		MCAL_GPIO_init(USARTxPORT[USARTx_ID], &Pin_Config);
		//Configure RX Pin
		Pin_Config = (GPIO_PinConfig_t){USARTxPINS[USARTx_ID][1],GPIO_INUPUT_FLOATING,GPIO_SPEED_2_MHZ};
		MCAL_GPIO_init(GPIO_PORTA, &Pin_Config);

		if (Global_USART_Config[USARTx_ID].HwFlowCtrl == USART_HW_FLOW_CTS
				|| Global_USART_Config[USARTx_ID].HwFlowCtrl == USART_HW_FLOW_RTS_CTS) {
			//Configure CTS Pin
			Pin_Config = (GPIO_PinConfig_t){USARTxPINS[USARTx_ID][2],GPIO_INUPUT_FLOATING,GPIO_SPEED_2_MHZ};
			MCAL_GPIO_init(USARTxPORT[USARTx_ID], &Pin_Config);

		}
		if (Global_USART_Config[USARTx_ID].HwFlowCtrl == USART_HW_FLOW_RTS
				|| Global_USART_Config[USARTx_ID].HwFlowCtrl == USART_HW_FLOW_RTS_CTS) {
			//Configure RTS Pin
			Pin_Config = (GPIO_PinConfig_t){USARTxPINS[USARTx_ID][3],GPIO_AF_OUTPUT_PP,GPIO_SPEED_2_MHZ};
			MCAL_GPIO_init(USARTxPORT[USARTx_ID], &Pin_Config);
		}
	}
}
//===================================================================================================

/*******************************************************************************
 *                            			ISR	  		                 			*
 *******************************************************************************/
void USART1_IRQHandler()         			/* USART1 global interrupt*/
{
	if(GP__callBackFunc != NULL){
		(*GP__callBackFunc[USART1_ID])();
	}
}
void USART2_IRQHandler()         			/* USART2 global interrupt*/
{
	if(GP__callBackFunc != NULL){
		(*GP__callBackFunc[USART2_ID])();
	}
}
void USART3_IRQHandler()         			/* USART3 global interrupt*/
{
	if(GP__callBackFunc != NULL){
		(*GP__callBackFunc[USART3_ID])();
	}
}
//==================================================================================================
