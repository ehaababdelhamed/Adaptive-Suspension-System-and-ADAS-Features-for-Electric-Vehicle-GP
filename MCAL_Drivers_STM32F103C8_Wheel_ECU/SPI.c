/******************************************************************************
 * Module: SPI
 * File Name: SPI.c
 * Description: Source file for SPI driver for Stm32f103x6
 * Author: Ehab Mohamed Abdelhamed
 ******************************************************************************/

/*******************************************************************************
 *                             Includes			                            	*
 *******************************************************************************/
#include "stm32f103x6.h"
#include "SPI.h"
#include "GPIO.h"

/*******************************************************************************
 *								Global Variables	                   			*
 *******************************************************************************/
static void (*GP__callBackFunc[NUM_OF_SPI_INSTANCES][NUM_OF_SPI_IRQ])(void)={
		{NULL,NULL,NULL},
		{NULL,NULL,NULL}
};

//SPI1 >> APB2 ,  SPI2 >> APB1
static SPI_Config_t Global_SPI_Config[NUM_OF_SPI_INSTANCES];
/* SPI1 :				SPI2 :
 * PA4  >> NSS			PB12 >> NSS
 * PA5 >> CLK			PB13 >> CLK
 * PA6 >> MISO			PB14 >> MISO
 * PA7 >> MOSI			PB15 >> MOSI
 * */


//USART GPIO PORT
static uint8 SPIxPORT[NUM_OF_SPI_INSTANCES]={
		GPIO_PORTA,
		GPIO_PORTB
};

//USART GPIO Pins
//{ NSS_PIN , CLK,PIN , MISO_PIN , MOSI_PIN}
static uint8 SPIxPINS[NUM_OF_SPI_INSTANCES][4]={
		{GPIO_PIN_4,GPIO_PIN_5,GPIO_PIN_6,GPIO_PIN_7},
		{GPIO_PIN_12,GPIO_PIN_13,GPIO_PIN_14,GPIO_PIN_15}
};

/*******************************************************************************
 *                           		Macros	                   					*
 *******************************************************************************/

#define GET_SPI_BASE_ADD(SPIx_ID) ((SPIx_ID) == SPI1_ID)? (SPI1) : (((SPIx_ID) == SPI2_ID)? SPI2 : NULL)

/*******************************************************************************
 *                            	Function Definition	                   			*
 *******************************************************************************/
/************************************************************************************
 * Function Name	: MCAL_SPI_init
 * Description		: Functional responsible for Initialize the SPI device by:
 * 1. Enable SPI Clock
 * 2. Select SPI mode [full-duplex  - half-duplex - simplex]
 * 3. Select Device Mode Master/Salve
 * 4. define 8- or 16-bit data frame format
 * 5. The data order (MSB-first or LSB-first).
 * 6. Select the CPOL and CPHA bits to define one of the four relationships between the
 * data transfer and the serial clock.
 * 7. Setup the SPI baud rate prescaler.
 * 8. Setup the NSS pin
 * 9. Enable SPI
 * Parameters (in)	: SPIConfig- pointer to SPI configuration parameters
 *  				  SPIx_ID - ID Number of SPI instance @REF : SPI_ID_define
 * Return value		: None
 * Note				: Stm32f103x6 Support only SPI1 and SPI2.
 * 					  We should initialize module before use the remain function.
 ************************************************************************************/
void MCAL_SPI_init(uint8 SPIx_ID,const SPI_Config_t *SPIConfig){
	volatile SPI_TypeDef * SPIx;		//to store the base address of SPI instance
	boolean error = FALSE;
	SPIx = GET_SPI_BASE_ADD(SPIx_ID);
	//Check if inputs is invalid
	if(SPIx == NULL ||SPIConfig == NULL ){
		error = TRUE;
	}
	if(error == FALSE){
		Global_SPI_Config[SPIx_ID] = (SPI_Config_t)(*SPIConfig);		//Save SPI configuration to use in another functions
		//1. Enable SPI Clock
		if( SPIx_ID == SPI1_ID ){
			RCC_SPI1_CLK_EN();
		}else if ( SPIx_ID == SPI2_ID ){
			RCC_SPI2_CLK_EN();
		}

		SPIx->CR1.bit.SPE = FALSE;		//Disable SPI to Configure it

		//2. Select SPI mode [full-duplex  - half-duplex - simplex]
		SPIx->CR1.bit.BIDIOE = ((SPIConfig->SPI_Mode)>>1)&(1U);
		SPIx->CR1.bit.RXONLY = (SPIConfig->SPI_Mode&(1U));

		//3. Select Device Mode Master/Salve
		SPIx->CR1.bit.MSTR = SPIConfig->Device_Mode;


		//4. define 8- or 16-bit data frame format
		SPIx->CR1.bit.DFF = SPIConfig->Data_Size;

		//5. The data order (MSB-first or LSB-first).
		SPIx->CR1.bit.LSBFIRST = SPIConfig->Data_Order;

		//6. Select the clock polarity and clock phase
		SPIx->CR1.bit.CPOL = SPIConfig->Clock_Polarity;
		SPIx->CR1.bit.CPHA = SPIConfig->Clock_Phase;

		//7. Setup the SPI baud rate prescaler.
		SPIx->CR1.bit.BR = SPIConfig->SPI_Prescaler;

		//8. Setup the NSS pin
		SPIx->CR1.bit.SSI_SSM = ((SPIConfig->NSS_Mode)>>1);
		SPIx->CR2.bit.SSOE = ((SPIConfig->NSS_Mode)&(1U));

		//9. Enable SPI
		SPIx->CR1.bit.SPE = TRUE;
	}
}

/************************************************************************************
 * Function Name	: MCAL_SPI_deInit
 * Description		: Functional responsible  reset all the SPI Registers
 * 					  Disable SPI interrupt from NVIC
 * Parameters (in)	: SPIx_ID - ID Number of SPI instance @REF : SPI_ID_define
 * Return value		: None
 ************************************************************************************/
void MCAL_SPI_deInit(uint8 SPIx_ID){
	volatile SPI_TypeDef * SPIx;		//to store the base address of SPI instance
	boolean error = FALSE;
	SPIx = GET_SPI_BASE_ADD(SPIx_ID);
	//Check if inputs is invalid
	if(SPIx == NULL){
		error = TRUE;
	}
	if(error == FALSE){
		//2. Reset  all SPI registers
		if( SPIx_ID == SPI1_ID ){
			RCC_SPI1_RESET();
			NVIC_IRQ35_SPI1_Disable();
		}else if ( SPIx_ID == SPI2_ID ){
			RCC_SPI2_RESET();
			NVIC_IRQ36_SPI2_Disable();
		}
	}
}

/************************************************************************************
 * Function Name	: MCAL_SPI_sendData
 * Description		: Functional responsible  to send data from SPI
 * Parameters (in)	: SPIx_ID - ID Number of SPI instance @REF : SPI_ID_define
 * 					  TxBuffer - Data to be send
 * 					  Polling - Select enable/disable polling
 * Return value		: None
 ************************************************************************************/
void MCAL_SPI_sendData(uint8 SPIx_ID,uint16 TxBuffer,SPI_PollingMechanism_e Polling){
	volatile SPI_TypeDef * SPIx;		//to store the base address of SPI instance
	boolean error = FALSE;
	SPIx = GET_SPI_BASE_ADD(SPIx_ID);
	//Check if inputs is invalid
	if(SPIx == NULL || Global_SPI_Config[SPIx_ID].SPI_Mode == SPI_MODE_SIMPLEX_RX ){
		error = TRUE;
	}
	if(error == FALSE){
		if(Global_SPI_Config[SPIx_ID].SPI_Mode == SPI_MODE_HALF_DUPLEX){
			/*
			 * The transfer direction (Input/Output) is selected by otherwise.
			 * the BIDIOE bit in the SPI_CR1 register.
			 * When this bit is 1, the data line is output otherwise it is input.
			 * */
			SPIx->CR1.bit.BIDIOE = TRUE;
		}
		if(Polling == SPI_POLLING_ENABLE){
			while(SPIx->SR.bit.TXE != TRUE);
		}
		SPIx->DR = TxBuffer;
	}
}


/************************************************************************************
 * Function Name	: MCAL_SPI_sendData
 * Description		: Functional responsible  to send data from SPI
 * Parameters (in)	: SPIx_ID - ID Number of SPI instance @REF : SPI_ID_define
 * 					  Polling - Select enable/disable polling
 * Return value		: RxBuffer - Data that is received
 ************************************************************************************/
uint16 MCAL_SPI_receiveData(uint8 SPIx_ID,SPI_PollingMechanism_e Polling){
	uint16 RxBuffer=0;		//to store received data
	volatile SPI_TypeDef * SPIx;		//to store the base address of SPI instance
	boolean error = FALSE;
	SPIx = GET_SPI_BASE_ADD(SPIx_ID);
	//Check if inputs is invalid
	if(SPIx == NULL || Global_SPI_Config[SPIx_ID].SPI_Mode == SPI_MODE_SIMPLEX_TX ){
		error = TRUE;
	}
	if(error == FALSE){
		if(Global_SPI_Config[SPIx_ID].SPI_Mode == SPI_MODE_HALF_DUPLEX){
			/*
			 * The transfer direction (Input/Output) is selected by
			 * the BIDIOE bit in the SPI_CR1 register.
			 * When this bit is 1, the data line is output otherwise it is input.
			 * */
			SPIx->CR1.bit.BIDIOE = FALSE;
		}

		if(Polling == SPI_POLLING_ENABLE){
			while(SPIx->SR.bit.RXNE != TRUE);
		}

		RxBuffer = ((SPIx)->DR);
	}

	return RxBuffer;
}


/************************************************************************************
 * Function Name	: MCAL_SPI_TX_RX
 * Description		: Functional responsible  to send and receive data from SPI
 * Parameters (in)	: SPIx_ID - ID Number of SPI instance @REF : SPI_ID_define
 * 					  TxBuffer - data to be send
 * 					  Polling - select enable/disable polling
 * Return value		: RxBuffer - data that is received
 ************************************************************************************/
uint16 MCAL_SPI_TX_RX(uint8 SPIx_ID,uint16 TxBuffer,SPI_PollingMechanism_e Polling){
	uint16 RxBuffer=0;		//to store received data
	volatile SPI_TypeDef * SPIx;		//to store the base address of SPI instance
	boolean error = FALSE;
	SPIx = GET_SPI_BASE_ADD(SPIx_ID);
	//Check if inputs is invalid
	if(SPIx == NULL || Global_SPI_Config[SPIx_ID].SPI_Mode != SPI_MODE_FULL_DUPLEX ){
		error = TRUE;
	}
	if(error == FALSE){

		if(Polling == SPI_POLLING_ENABLE){
			while(SPIx->SR.bit.TXE != TRUE);
		}

		(SPIx)->DR = TxBuffer;

		if(Polling == SPI_POLLING_ENABLE){
			while(SPIx->SR.bit.RXNE != TRUE);
		}

		RxBuffer = (SPIx)->DR;
	}
	return RxBuffer;
}


/************************************************************************************
 * Function Name	: MCAL_SPI_GPIO_SET_PINS
 * Description		: Functional responsible to setup GPIO pin for SPI
 * Parameters (in)	: SPIx_ID - ID Number of SPI instance @REF : SPI_ID_define
 * Return value		: None
 ************************************************************************************/
void MCAL_SPI_GPIO_SET_PINS(uint8 SPIx_ID){
	GPIO_PinConfig_t Pin_Config;
	volatile SPI_TypeDef * SPIx;		//to store the base address of SPI instance
	boolean error = FALSE;
	SPIx = GET_SPI_BASE_ADD(SPIx_ID);
	//Check if inputs is invalid
	if(SPIx == NULL){
		error = TRUE;
	}
	if(error == FALSE){
		//1-Configure NSS Pin
		switch(Global_SPI_Config[SPIx_ID].NSS_Mode){
		case SPI_NSS_HW_MultiMaster_OUPTUT_ENABLE:
			Pin_Config = (GPIO_PinConfig_t){SPIxPINS[SPIx_ID][0],GPIO_AF_OUTPUT_PP,GPIO_SPEED_2_MHZ};
			MCAL_GPIO_init(SPIxPORT[SPIx_ID], &Pin_Config);
			break;
		case SPI_NSS_HW_SLAVE: //SPI_NSS_HW_MultiMaster_OUPTUT_DISABLE == SPI_NSS_HW_SLAVE
			Pin_Config = (GPIO_PinConfig_t){SPIxPINS[SPIx_ID][0],GPIO_INUPUT_FLOATING,GPIO_SPEED_INPUT};
			MCAL_GPIO_init(SPIxPORT[SPIx_ID], &Pin_Config);
			break;
		default:
			break;
		}

		//2-Configure CLK Pin
		switch(Global_SPI_Config[SPIx_ID].Device_Mode){
		case SPI_MASTER:
			Pin_Config = (GPIO_PinConfig_t){SPIxPINS[SPIx_ID][1],GPIO_AF_OUTPUT_PP,GPIO_SPEED_2_MHZ};
			MCAL_GPIO_init(SPIxPORT[SPIx_ID], &Pin_Config);
			break;
		case SPI_SLAVE:
			Pin_Config = (GPIO_PinConfig_t){SPIxPINS[SPIx_ID][1],GPIO_INUPUT_FLOATING,GPIO_SPEED_INPUT};
			MCAL_GPIO_init(SPIxPORT[SPIx_ID], &Pin_Config);
			break;
		default:
			break;
		}

		//3-Configure MISO Pin
		switch(Global_SPI_Config[SPIx_ID].Device_Mode){
		case SPI_MASTER:
			if(Global_SPI_Config[SPIx_ID].SPI_Mode == SPI_MODE_FULL_DUPLEX || Global_SPI_Config[SPIx_ID].SPI_Mode == SPI_MODE_SIMPLEX_RX){
				Pin_Config = (GPIO_PinConfig_t){SPIxPINS[SPIx_ID][2],GPIO_INUPUT_FLOATING,GPIO_SPEED_INPUT};
				MCAL_GPIO_init(SPIxPORT[SPIx_ID], &Pin_Config);
			}
			break;
		case SPI_SLAVE:
			if(Global_SPI_Config[SPIx_ID].SPI_Mode != SPI_MODE_SIMPLEX_RX){
				Pin_Config = (GPIO_PinConfig_t){SPIxPINS[SPIx_ID][2],GPIO_AF_OUTPUT_PP,GPIO_SPEED_2_MHZ};
				MCAL_GPIO_init(SPIxPORT[SPIx_ID], &Pin_Config);
			}
			break;
		default:
			break;
		}

		//4-Configure MOSI Pin
		switch(Global_SPI_Config[SPIx_ID].Device_Mode){
		case SPI_MASTER:
			if( Global_SPI_Config[SPIx_ID].SPI_Mode != SPI_MODE_SIMPLEX_RX){
				Pin_Config = (GPIO_PinConfig_t){SPIxPINS[SPIx_ID][3],GPIO_AF_OUTPUT_PP,GPIO_SPEED_2_MHZ};
				MCAL_GPIO_init(SPIxPORT[SPIx_ID], &Pin_Config);
			}
			break;
		case SPI_SLAVE:
			if(Global_SPI_Config[SPIx_ID].SPI_Mode == SPI_MODE_FULL_DUPLEX ||  Global_SPI_Config[SPIx_ID].SPI_Mode == SPI_MODE_SIMPLEX_RX){
				Pin_Config = (GPIO_PinConfig_t){SPIxPINS[SPIx_ID][3],GPIO_INUPUT_FLOATING,GPIO_SPEED_INPUT};
				MCAL_GPIO_init(SPIxPORT[SPIx_ID], &Pin_Config);
			}
			break;
		default:
			break;
		}
	}
}


/************************************************************************************
 * Function Name	: MCAL_SPI_enableInterrupt
 * Description		: Functional responsible to Enable SPI interrupt
 * Parameters (in)	: SPIx_ID - ID Number of SPI instance @REF : SPI_ID_define
 * 					  IRQ_Source - Source of interrupt @REF : SPI_IRQ_Source_define
 * Return value		: None
 ************************************************************************************/
void MCAL_SPI_enableInterrupt(uint8 SPIx_ID,uint8 IRQ_Source){
	volatile SPI_TypeDef * SPIx;		//to store the base address of SPI instance
	boolean error = FALSE;
	SPIx = GET_SPI_BASE_ADD(SPIx_ID);
	//Check if inputs is invalid
	if(SPIx == NULL){
		error = TRUE;
	}
	if(error == FALSE){
		if( SPIx_ID == SPI1_ID ){
			NVIC_IRQ35_SPI1_Enable();
		}else if ( SPIx_ID == SPI2_ID ){
			NVIC_IRQ36_SPI2_Enable();
		}
		MCAL_SPI_enable(SPIx_ID);
		SET_BIT((SPIx->CR2.ALL_REG),IRQ_Source);
	}
}



/************************************************************************************
 * Function Name	: MCAL_SPI_DisableInterrupt
 * Description		: Functional responsible to Disable SPI interrupt
 * Parameters (in)	: SPIx_ID - ID Number of SPI instance @REF : SPI_ID_define
 * 					  IRQ_Source - Source of interrupt @REF : SPI_IRQ_Source_define
 * Return value		: None
 ************************************************************************************/
void MCAL_SPI_disableInterrupt(uint8 SPIx_ID,uint8 IRQ_Source){
	volatile SPI_TypeDef * SPIx;		//to store the base address of SPI instance
	boolean error = FALSE;
	SPIx = GET_SPI_BASE_ADD(SPIx_ID);
	//Check if inputs is invalid
	if(SPIx == NULL){
		error = TRUE;
	}
	if(error == FALSE){
		CLEAR_BIT((SPIx->CR2.ALL_REG),IRQ_Source);
	}
}


/************************************************************************************
 * Function Name	: MCAL_SPI_setCallBackFunc
 * Description		: Functional responsible to set function that is called when IRQ occur
 * Parameters (in)	: SPIx_ID - ID Number of SPI instance @REF : SPI_ID_define
 * 					  IRQ_Source - Source of interrupt @REF : SPI_IRQ_Source_define
 * 					  callBackFunc - pointer to call back function
 * Return value		: None
 ************************************************************************************/
void MCAL_SPI_setCallBackFunc(uint8 SPIx_ID,uint8 IRQ_Source, void (*callBackFunc)() ){
	volatile SPI_TypeDef * SPIx;		//to store the base address of SPI instance
	boolean error = FALSE;
	SPIx = GET_SPI_BASE_ADD(SPIx_ID);
	//Check if inputs is invalid
	if(SPIx == NULL){
		error = TRUE;
	}
	if(error == FALSE){
		switch(IRQ_Source){
		case  SPI_IRQ_Source_TXE:
			GP__callBackFunc[SPIx_ID][Source_TXE] = callBackFunc;
			break;
		case  SPI_IRQ_Source_RXNE:
			GP__callBackFunc[SPIx_ID][Source_RXNE] = callBackFunc;
			break;
		case  SPI_IRQ_Source_ERR:
			GP__callBackFunc[SPIx_ID][Source_ERR] = callBackFunc;
			break;
		default:
			break;
		}
	}
}


/************************************************************************************
 * Function Name	: MCAL_SPI_enable
 * Description		: Functional responsible to enable SPI
 * Parameters (in)	: SPIx_ID - ID Number of SPI instance @REF : SPI_ID_define
 * Return value		: None
 ************************************************************************************/
void MCAL_SPI_enable(uint8 SPIx_ID){
	volatile SPI_TypeDef * SPIx;		//to store the base address of SPI instance
	boolean error = FALSE;
	SPIx = GET_SPI_BASE_ADD(SPIx_ID);
	//Check if inputs is invalid
	if(SPIx == NULL){
		error = TRUE;
	}
	if(error == FALSE){
		//Enable SPI
		SPIx->CR1.bit.SPE = TRUE;
	}
}


/************************************************************************************
 * Function Name	: MCAL_SPI_disable
 * Description		: Functional responsible to disable SPI
 * Parameters (in)	: SPIx_ID - ID Number of SPI instance @REF : SPI_ID_define
 * Return value		: None
 ************************************************************************************/
void MCAL_SPI_disable(uint8 SPIx_ID){
	volatile SPI_TypeDef * SPIx;		//to store the base address of SPI instance
	boolean error = FALSE;
	SPIx = GET_SPI_BASE_ADD(SPIx_ID);
	//Check if inputs is invalid
	if(SPIx == NULL){
		error = TRUE;
	}
	if(error == FALSE){
		//Disable SPI
		SPIx->CR1.bit.SPE = FALSE;
	}
}


/************************************************************************************
 * Function Name	: MCAL_SPI_waitBusyFlag
 * Description		: Functional responsible to wait until busy flag is true
 * Parameters (in)	: SPIx_ID - ID Number of SPI instance @REF : SPI_ID_define
 * Return value		: None
 ************************************************************************************/
void MCAL_SPI_waitBusyFlag(uint8 SPIx_ID){
	volatile SPI_TypeDef * SPIx;		//to store the base address of SPI instance
	boolean error = FALSE;
	SPIx = GET_SPI_BASE_ADD(SPIx_ID);
	//Check if inputs is invalid
	if(SPIx == NULL){
		error = TRUE;
	}
	if(error == FALSE){
		while(SPIx->SR.bit.BSY != TRUE);
	}
}


//===============================================================================================
/*******************************************************************************
 *                            			ISR	  		                 			*
 *******************************************************************************/
void SPI1_IRQHandler(){

	if(SPI1->SR.bit.RXNE == TRUE){
		if(GP__callBackFunc[SPI1_ID][Source_RXNE] != NULL){
			(*GP__callBackFunc[SPI1_ID][Source_RXNE])();
		}
	}

	if(SPI1->SR.bit.TXE == TRUE){
		if(GP__callBackFunc[SPI1_ID][Source_TXE] != NULL){
			(*GP__callBackFunc[SPI1_ID][Source_TXE])();
		}
	}
}


void SPI2_IRQHandler(){
	if(SPI1->SR.bit.RXNE == TRUE){
		if(GP__callBackFunc[SPI2_ID][Source_RXNE] != NULL){
			(*GP__callBackFunc[SPI2_ID][Source_RXNE])();
		}
	}

	if(SPI2->SR.bit.TXE == TRUE){
		if(GP__callBackFunc[SPI2_ID][Source_TXE] != NULL){
			(*GP__callBackFunc[SPI2_ID][Source_TXE])();
		}
	}
}
//==================================================================================================
