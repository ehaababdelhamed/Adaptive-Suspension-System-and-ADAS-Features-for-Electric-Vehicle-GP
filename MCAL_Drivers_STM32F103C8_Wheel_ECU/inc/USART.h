/******************************************************************************
 * Module: USART
 * File Name: USART.h
 * Description: Header file for USART driver for Stm32f103x6
 * Author: Ehab Mohamed Abdelhamed
 ******************************************************************************/
#ifndef INC_USART_H_
#define INC_USART_H_

/*******************************************************************************
 *                             Includes			                            	*
 *******************************************************************************/
#include "Common_Macros.h"
#include "Platform_Types.h"

/*******************************************************************************
 *                             	USART Data Types                             	*
 *******************************************************************************/
//@REF: USART_Mode_e
typedef enum{
	USART_MODE_RX=1,
	USART_MODE_TX,
	USART_MODE_TX_RX
}USART_Mode_e;

//@REF: USART_BoudRate_e
typedef enum{
	USART_BAUDRATE_2400=2400  		, USART_BAUDRATE_4800=4800,
	USART_BAUDRATE_9600=9600  		, USART_BAUDRATE_19200=19200,
	USART_BAUDRATE_57600=57600		, USART_BAUDRATE_115200=115200,
	USART_BAUDRATE_921600=921600	, USART_BAUDRATE_2250000=2250000,
	USART_BAUDRATE_4500000=4500000
}USART_BoudRate_e;

//@REF: USART_DateLength_e
typedef enum{
	USART_DATE_lENGTH_8_BITS,
	USART_DATE_lENGTH_9_BITS
}USART_DateLength_e;

//@REF: USART_Parity
typedef enum{
	USART_DISAPLE_PARITY,
	USART_EVEN_PARITY=2,
	USART_ODD_PARITY
}USART_Parity_e;

//@REF: USART_StopBit
typedef enum{
	USART_STOP_1_BIT,
	USART_STOP_HALF_BIT,
	USART_STOP_2_BIT,
	USART_STOP_1_HALF_BIT
}USART_StopBit_e;

//@REF: USART_HwFlowCtrl_e mechanism
typedef enum{
	USART_HW_FLOW_CTRL_NONE,
	USART_HW_FLOW_RTS,
	USART_HW_FLOW_CTS,
	USART_HW_FLOW_RTS_CTS
}USART_HwFlowCtrl_e;

typedef struct{
	USART_Mode_e USART_Mode;			//Specifies USART Mode as TX/RX
										//This parameter can be a value of @REF: USART_Mode
	USART_BoudRate_e Boud_Rate;			//Specifies baud rate of USART
										//This parameter can be a value of @REF: USART_BoudRate_e
	USART_DateLength_e Data_Length;		//Specifies the Number of data bits of USART frame
										//This parameter can be a value of @REF: USART_DataBit
	USART_Parity_e Parity;				//Specifies the parity of USART  frame
										//This parameter can be a value of @REF: USART_Parity
	USART_StopBit_e Stop_Bit;			//Specifies the number of stop bits of USART  frame
										//This parameter can be a value of @REF: USART_StopBit
	USART_HwFlowCtrl_e HwFlowCtrl;		//Specifies Hardware flow control of USART
										//This parameter can be a value of @REF: USART_HwFlowCtrl_e
}USART_Config_t;

typedef enum{
	USART_POOLING_DISABLE,
	USART_POOLING_ENABLE
}USART_PoolingMechanism_e;

//@REF: USART_IRQ_Types
typedef enum{
	USART_IRQ_ENABLE_RXNE=5,
	USART_IRQ_ENABLE_TC,
	USART_IRQ_ENABLE_TXE,
	USART_IRQ_ENABLE_PE,
}USART_IRQ_e;

typedef enum{
	USART1_ID,
	USART2_ID,
	USART3_ID
}USART_Instance_e;


/*******************************************************************************
 *                             	USART Definition                             	*
 *******************************************************************************/
#define NUM_OF_UART_INSTANCES (3U)
#define NUM_OF_UART_IRQ (4U)
#define IRQ_SHIFT (5U)
/* USART1 :				USART2 :			USART3 :
 * PA9  >> TX			PA2 >> TX			PB10  >> TX
 * PA10 >> RX			PA3 >> RX			PB11 >> RX
 * PA11 >> CTS			PA0 >> CTS			PB13 >> CTS
 * PA12 >> RTS			PA1 >> RTS			PB14 >> RTS
 * */

/*******************************************************************************
 *					APIs Supported by "MCAL USART DRIVER" 	                   *
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
 * Note				: Stm32f103x6 Support only UART1 and UART2
 ************************************************************************************/
void MCAL_USART_init(USART_Instance_e USARTx_ID,USART_Config_t *USARTConfig);

/************************************************************************************
 * Function Name	: MCAL_USART_deInit
 * Description		: Functional responsible for reset the all USART register.
 * 					  Disable USART.
 * Parameters (in)	: USARTx_ID - ID Number of UART instance
 * Return value		: None
 * Note				: None
 ************************************************************************************/
void MCAL_USART_deInit(USART_Instance_e USARTx);

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
void MCAL_USART_setCallBackFunc(USART_Instance_e USARTx,USART_IRQ_e IRQx,void (*callBackFunc)());

/************************************************************************************
 * Function Name	: MCAL_USART_sendData
 * Description		: Functional responsible to send data by USART.
 * Parameters (in)	: USARTx_ID - ID Number of UART instance
 *  				  TXD_Buffer : data to be send by USART
 *  				  Pooling - Select [POOLING_ENABLE - POOLING_DISABLE]
 * Return value		: None
 * Note				: None
 ************************************************************************************/
void MCAL_USART_sendData(USART_Instance_e USARTx,uint16 Data,USART_PoolingMechanism_e Pooling);

/************************************************************************************
 * Function Name	: MCAL_USART_receiveData
 * Description		: Functional responsible to receive data by USART.
 * Parameters (in)	: USARTx_ID - ID Number of UART instance
 *  				  Pooling - Select [POOLING_ENABLE - POOLING_DISABLE]
 * Return value		: Data to be received
 * Note				: None
 ************************************************************************************/
uint16 MCAL_USART_receiveData(USART_Instance_e USARTx,USART_PoolingMechanism_e Pooling);

/************************************************************************************
 * Function Name	: MCAL_USART_WAIT_TC
 * Description		: Functional responsible to wait until TC flag is set;
 * Parameters (in)	: USARTx_ID - ID Number of UART instance
 * Return value		: Data to be received
 * Note				: None
 ************************************************************************************/
void MCAL_USART_WAIT_TC(USART_Instance_e USARTx_ID);

/************************************************************************************
 * Function Name	: MCAL_USART_GPIO_SET_PINS
 * Description		: Functional responsible to set GPIO pins of USART.
 * Parameters (in)	: USARTx_ID - ID Number of UART instance
 * Return value		: Character to be received
 * Note				: None
 ************************************************************************************/
void MCAL_USART_GPIO_SET_PINS(USART_Instance_e USARTx_ID);


/************************************************************************************
 * Function Name	: MCAL_USART_sendString
 * Description		: Functional responsible to send string by USART.
 * Parameters (in)	: data : pointer to start character of string to be send by USART
 * Return value		: None
 * Note				: None
 ************************************************************************************/
//void MCAL_USART_sendString(char* data);
/************************************************************************************
 * Function Name		: MCAL_USART_receiveCharacter
 * Description			: Functional responsible to receive string by USART.
 * Parameters (in/out)	: string : pointer to store received string
 * Return value			: None
 * Note					: None
 ************************************************************************************/
//void MCAL_USART_receiveString(char *string);
/************************************************************************************
 * Function Name		: MCAL_USART_sendSInteger
 * Description			: Functional responsible to Send integer by USART.
 * Parameters (in/out)	: string : pointer to store received string
 * Return value			: None
 * Note					: None
 ************************************************************************************/
//void MCAL_USART_sendSInteger(uint32 data);
/************************************************************************************
 * Function Name		: MCAL_USART_ReceiveInteger
 * Description			: Functional responsible to receive string by USART.
 * Parameters (in/out)	: string : pointer to store received string
 * Return value			: None
 * Note					: None
 ************************************************************************************/
//uint32 MCAL_USART_ReceiveInteger();

#endif /* INC_USART_H_ */
