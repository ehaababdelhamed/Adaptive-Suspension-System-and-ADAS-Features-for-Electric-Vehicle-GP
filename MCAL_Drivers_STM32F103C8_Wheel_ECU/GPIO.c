/******************************************************************************
 * Module: GPIO
 * File Name: GPIO.c
 * Description: Source file for GPIO driver for stm32f103x6
 * Author: Ehab Mohamed Abdelhamed
 ******************************************************************************/

/*******************************************************************************
 *                             Includes			                            	*
 *******************************************************************************/
#include <GPIO.h>
#include "stm32f103x6.h"


/*******************************************************************************
 *                            Private Function Prototypes	                   *
 *******************************************************************************/
static GPIO_TypeDef* GPIO_GetPortAdd(uint8 PortNum);


/*******************************************************************************
 *                            	Function Definition	                   			*
 *******************************************************************************/

/************************************************************************************
 * Function Name	: MCAL_GPIO_init
 * Description		: Function to initialize the Specific pin in specific GPIO module.
 * Parameters (in)	: GPIO_Portx- Number of Specific instance of GPIO @REF: GPIO_NUMBER
 * 					  pinConfig - Pointer to pin configuration date
 * Return value		: None
 * Note				: Stm32f103x6 supported GPIOA,GPIOB,GPIOC,GPIOD,GPIOE
 * 					  but LQF48 Package supported fully GPIOA,GPIOB
 * 					  and partially GPIOC[13-15] and GPIOD[0-1]
 ************************************************************************************/

void MCAL_GPIO_init(uint8 GPIO_Portx,GPIO_PinConfig_t *pinConfig){
	volatile uint32 *ConfigReg = NULL;	//For select CRH or CHL
	uint8 PinModeConfig=0;				//to shift mode number for output modes
	uint8 CRHL_Shift=0;					//to Calculate the shift corresponding to the pin
	boolean error=FALSE;				//to check the if inputs is valid
	GPIO_TypeDef *GPIOx;				//Store Address of GPIOx
	GPIOx=GPIO_GetPortAdd(GPIO_Portx);
	if(GPIOx == NULL || pinConfig->Pin_Num < GPIO_PIN_0 || pinConfig->Pin_Num > GPIO_PIN_15){
		error=TRUE;
	}
	if(error == FALSE){
		//Port configuration register low (GPIOx_CRL) configure pins [0-7]
		//Port configuration register high (GPIOx_CRH) configure pins [8-15]
		(pinConfig->Pin_Num <= GPIO_PIN_7)? (ConfigReg = &(GPIOx->CRL)): (ConfigReg = &(GPIOx->CRH));

		//Calculate the shift corresponding to the pin
		//Shifted pin : EX: pin8 >> pin0 in CRH
		CRHL_Shift=(pinConfig->Pin_Num%GPIO_PIN_8)*MODE_SHIFT;

		//CLEAR MODEx:2 CNFx:2 corresponding to the pin
		*ConfigReg &= ~(0xf<< CRHL_Shift) ;
		//Check in pin is at any output mode
		if(pinConfig->Pin_Mode >= GPIO_OUTPUT_PP){
			//Shifted Mode : EX: GPIO_OUTPUT_OD= 0b100 >> = 0b10
			PinModeConfig = ((pinConfig->Pin_Mode-OUTPUT_MODES_SHIFT) << CNF_SHIFT) | (pinConfig->Pin_Speed & 0x03);
		}else{	//if Pin is input
			switch(pinConfig->Pin_Mode){
			case GPIO_INPUT_ANALOG:
			case GPIO_INUPUT_FLOATING:
				PinModeConfig = ((pinConfig->Pin_Mode << CNF_SHIFT) & 0x0f);
				break;
			case GPIO_INPUT_PU:
				PinModeConfig = ((GPIO_INPUT_PU << CNF_SHIFT) & 0x0f);
				//For Enable Pull Up >> ODRx =1
				SET_BIT(GPIOx->ODR,pinConfig->Pin_Num);
				break;
			case GPIO_INPUT_PD:
				//For Enable Pull Down >> ODRx =0
				PinModeConfig = (((GPIO_INPUT_PD-1) << CNF_SHIFT) & 0x0f);
				CLEAR_BIT(GPIOx->ODR,pinConfig->Pin_Num);
				break;
			default:
				break;
			}
		}
		//Set The required speed in MODE [2 bits] and the required pin mode in CNF [2 bits]
		*ConfigReg|=( PinModeConfig << (CRHL_Shift));
	}
}

/************************************************************************************
 * Function Name	: MCAL_GPIO_deInit
 * Description		: Function to reset all registers of specific GPIO.
 * Parameters (in)	: GPIO_Portx- Number of Specific instance of GPIO @REF: GPIO_NUMBER
 * Return value		: None
 * Note				: None
 ************************************************************************************/

void MCAL_GPIO_deInit(uint8 GPIO_Portx){
	//Assign all registers to reset value
	/*
	GPIOx->CRL = 0x44444444;
	GPIOx->CRH = 0x44444444;
	GPIOx->BRR=  0x00000000;
	GPIOx->BSRR= 0x00000000;
	GPIOx->LCKR= 0x00000000;
	GPIOx->ODR=  0x00000000;
	//GPIOx->IDR (Read Only)
	 */
	//Use RCC Module to reset
	if(GPIO_Portx == GPIO_PORTA){
		RCC_GPIOA_RESET();
	}else if(GPIO_Portx == GPIO_PORTB){
		RCC_GPIOB_RESET();
	}else if(GPIO_Portx == GPIO_PORTC){
		RCC_GPIOC_RESET();
	}else if(GPIO_Portx == GPIO_PORTD){
		RCC_GPIOD_RESET();
	}else if(GPIO_Portx == GPIO_PORTE){
		RCC_GPIOE_RESET();
	}
}


/************************************************************************************
 * Function Name	: MCAL_GPIO_readPin
 * Description		: Function to read specific pin of specific GPIO.
 * Parameters (in)	: GPIO_Portx- Number of Specific instance of GPIO @REF: GPIO_NUMBER
 * 					  Pin_Num	- Pin number according @REF:GPIO_PINS_NUMBER
 * Return value		: PinLevelType according  @REF:GPIO_PIN_LEVEL
 * Note				: None
 ************************************************************************************/
uint8 MCAL_GPIO_readPin(uint8 GPIO_Portx,uint8 Pin_Num){
	uint8 PinLevelType=PIN_LOW;			//To store the pin value
	boolean error=FALSE;				//To check the if inputs is valid
	GPIO_TypeDef *GPIOx;				//Store Address of GPIOx
	GPIOx=GPIO_GetPortAdd(GPIO_Portx);
	if(GPIOx == NULL || Pin_Num< GPIO_PIN_0 || Pin_Num > GPIO_PIN_15){
		error=TRUE;
	}
	//if input valid >> Read the pin value from GPIO_IDR Register
	if(error == FALSE){
		if(BIT_IS_SET(GPIOx->IDR,Pin_Num) == PIN_LOW ){
			PinLevelType = PIN_LOW;
		}else{
			PinLevelType = PIN_HIGH;
		}
	}
	return PinLevelType;
}


/************************************************************************************
 * Function Name	:  MCAL_GPIO_readPort
 * Description		: Function to read all port of specific GPIO.
 * Parameters (in)	: GPIO_Portx- Number of Specific instance of GPIO @REF: GPIO_NUMBER
 * Return value		: PortLevelType
 * Note				: None
 ************************************************************************************/
uint16 MCAL_GPIO_readPort(uint8 GPIO_Portx){
	uint8 PortLevelType=0;
	boolean error=FALSE;				//to check the if inputs is valid
	GPIO_TypeDef *GPIOx;				//Store Address of GPIOx
	GPIOx=GPIO_GetPortAdd(GPIO_Portx);
	if(GPIOx == NULL ){
		error=TRUE;
	}
	//if input valid >> Read the port value from GPIO_IDR Register
	if(error == FALSE){
		PortLevelType = (uint16)GPIOx->IDR;
	}
	return PortLevelType;
}

/************************************************************************************
 * Function Name	: MCAL_GPIO_writePin
 * Description		: Function to write in specific pin of specific GPIO.
 * Parameters (in)	: GPIO_Portx- Number of Specific instance of GPIO @REF: GPIO_NUMBER
 * 					  Pin_Num	- Pin number according @REF:GPIO_PINS_NUMBER
 * 					  Pin_Value - Pin value according @REF:GPIO_PIN_LEVEL
 * Return value		: None
 * Note				: None
 ************************************************************************************/
void MCAL_GPIO_writePin(uint8 GPIO_Portx,uint8 Pin_Num,uint8 Pin_Value){
	boolean error=FALSE;				//to check the if inputs is valid
	GPIO_TypeDef *GPIOx;				//Store Address of GPIOx
	GPIOx=GPIO_GetPortAdd(GPIO_Portx);
	if(GPIOx == NULL || Pin_Num < GPIO_PIN_0 || Pin_Num > GPIO_PIN_15){
		error=TRUE;
	}
	//if input valid >> Write the value in the required pin
	if(error == FALSE){
		if(Pin_Value == PIN_HIGH){
			/* Port bit set/reset register (GPIOx_BSRR)
			 * Bits 15:0 BSy: Port x Set bit y (y= 0 .. 15)
			 * These bits are write-only and can be accessed in Word mode only.
			 * 0: No action on the corresponding ODRx bit
			 * 1: Set the corresponding ODRx bit*/
			//GPIOx->BSRR = (1<<Pin_Num);
			SET_BIT(GPIOx->ODR,Pin_Num);
		}else if(Pin_Value == PIN_LOW){
			/* Port bit reset register (GPIOx_BRR)
			 * Bits 15:0 BRy: Port x Reset bit y (y= 0 .. 15)
			 * These bits are write-only and can be accessed in Word mode only.
			 * 0: No action on the corresponding ODRx bit
			 * 1: Reset the corresponding ODRx bit*/
			//GPIOx->BRR = (1<<Pin_Num);
			CLEAR_BIT(GPIOx->ODR,Pin_Num);
		}
	}
}

/************************************************************************************
 * Function Name	: MCAL_GPIO_writePort
 * Description		: Function to write in all port of specific GPIO.
 * Parameters (in)	: GPIO_Portx- Number of Specific instance of GPIO @REF: GPIO_NUMBER
 * 					  Port_Value - Port value
 * Return value		: None
 * Note				: None
 ************************************************************************************/
void MCAL_GPIO_writePort(uint8 GPIO_Portx,uint16 Port_Value){
	boolean error=FALSE;				//to check the if inputs is valid
	GPIO_TypeDef *GPIOx;				//Store Address of GPIOx
	GPIOx=GPIO_GetPortAdd(GPIO_Portx);
	//if input valid >> Write the value in the required port
	if(GPIOx == NULL){
		error=TRUE;
	}
	if(error == FALSE){
		GPIOx->ODR = (uint16)(Port_Value);
	}
}



/************************************************************************************
 * Function Name	: MCAL_GPIO_writeGroub
 * Description		: Function to write in Group Of pins of specific GPIO.
 * Parameters (in)	: GPIO_Portx- Number of Specific instance of GPIO @REF: GPIO_NUMBER
 * 					  Port_Value - Port value
 * 					  offset - To select the position of from LSB
					  mask 	 - To Mask group of bits From start position
 * Return value		: None
 * Note				: None
 ************************************************************************************/
void MCAL_GPIO_writeGroub(uint8 GPIO_Portx,uint16 mask,uint8 offset,uint16 value){
	boolean error=FALSE;				//to check the if inputs is valid
	GPIO_TypeDef *GPIOx;				//Store Address of GPIOx
	GPIOx=GPIO_GetPortAdd(GPIO_Portx);
	//if input valid >> Write the value in the required port
	if(GPIOx == NULL){
		error=TRUE;
	}
	if(error == FALSE){
		GPIOx->ODR &= ~(mask<<offset);
		GPIOx->ODR |= (value & mask) << offset;
	}
}

/************************************************************************************
 * Function Name	: MCAL_GPIO_readGroup
 * Description		: Function to read Group Of pins of specific GPIO.
 * Parameters (in)	: GPIO_Portx- Number of Specific instance of GPIO @REF: GPIO_NUMBER
 * 					  Port_Value - Port value
 * 					  offset - To select the position of from LSB
					  mask 	 - To Select Pins to read
 * Return value		: None
 * Note				: None
 ************************************************************************************/
uint16 MCAL_GPIO_readGroup(uint8 GPIO_Portx,uint16 mask,uint8 offset){
	uint8 PortLevelType=0;
	boolean error=FALSE;				//to check the if inputs is valid
	GPIO_TypeDef *GPIOx;				//Store Address of GPIOx
	GPIOx=GPIO_GetPortAdd(GPIO_Portx);
	if(GPIOx == NULL ){
		error=TRUE;
	}
	//if input valid >> Read the port value from GPIO_IDR Register
	if(error == FALSE){
		PortLevelType = (((uint16)GPIOx->IDR & (mask<< offset)) >> offset);
	}
	return PortLevelType;
}

/************************************************************************************
 * Function Name	: MCAL_GPIO_togglePin
 * Description		: Function to toggle specific pin of specific GPIO.
 * Parameters (in)	: GPIO_Portx- Number of Specific instance of GPIO @REF: GPIO_NUMBER
 * 					  Pin_Num	- Pin number according @REF:GPIO_PINS_NUMBER
 * Return value		: None
 * Note				: None
 ************************************************************************************/
void MCAL_GPIO_togglePin(uint8 GPIO_Portx,uint8 Pin_Num){
	boolean error=FALSE;				//to check the if inputs is valid
	GPIO_TypeDef *GPIOx;				//Store Address of GPIOx
	GPIOx=GPIO_GetPortAdd(GPIO_Portx);
	//if input valid >> toggle the required pin
	if(GPIOx == NULL || Pin_Num< GPIO_PIN_0 || Pin_Num > GPIO_PIN_15){
		error=TRUE;
	}
	if(error == FALSE){
			TOGGLE_BIT(GPIOx->ODR,Pin_Num);
	}
}


/************************************************************************************
 * Function Name	: MCAL_GPIO_lockPin
 * Description		: Function to frozen I/O configuration pin
 * Parameters (in)	: GPIO_Portx- Number of Specific instance of GPIO @REF: GPIO_NUMBER
 * 					  Pin_Num	- Pin number according @REF:GPIO_PINS_NUMBER
 * Return value		: Locking state (LOCKING_SUCCESS,LOCKING_ERROR)    @REF:GPIO_LOCKING_STATE
 * Note				: None
 ************************************************************************************/
boolean MCAL_GPIO_lockPin(uint8 GPIO_Portx,uint8 Pin_Num){
	boolean error=FALSE;				//to check the if inputs is valid
	GPIO_TypeDef *GPIOx;				//Store Address of GPIOx
	uint8 temp;
	boolean lockStatus = LOCKING_ERROR;
	GPIOx=GPIO_GetPortAdd(GPIO_Portx);
	if(GPIOx == NULL || Pin_Num< GPIO_PIN_0 || Pin_Num > GPIO_PIN_15){
		error=TRUE;
	}
	if(error == FALSE){
		/*Port configuration lock register (GPIOx_LCKR)
		 * Bit 16 LCKK[16]: Lock key
		 * This bit can be read any time. It can only be modified using the Lock Key Writing Sequence.
		 * 0: Port configuration lock key not active
		 * 1: Port configuration lock key active. GPIOx_LCKR register is locked until the next reset.
		 * LOCK key writing sequence:
		 * Write 1
		 * Write 0
		 * Write 1
		 * Read 0
		 * Read 1 (this read is optional but confirms that the lock is active)
		 * Note: During the LOCK Key Writing sequence, the value of LCK[15:0] must not change.
		 * Any error in the lock sequence will abort the lock.
		 *
		 * Bits 15:0 LCKy: Port x Lock bit y (y= 0 .. 15)
		 * These bits are read write but can only be written when the LCKK bit is 0.
		 * 0: Port configuration not locked
		 * 1: Port configuration locked.*/
		temp =(1<<16) |(1<<Pin_Num);
		GPIOx->LCKR = temp; 		//Write 1 >> bit16
		GPIOx->LCKR = (1<<Pin_Num); //Write 0 >> bit16
		GPIOx->LCKR = temp; 		//Write 1 >> bit16
		GPIOx->LCKR = (1<<Pin_Num); //Write 0 >> bit16
		if((GPIOx->LCKR & (1<<16)) == LOCKING_ERROR ){
			lockStatus = LOCKING_ERROR;
		}else{
			lockStatus = LOCKING_SUCCESS;
		}
	}
	return lockStatus;
}

//=============================================================================================
/*Private Function to Get the Address of GPIOx According to input port number*/
static GPIO_TypeDef* GPIO_GetPortAdd(uint8 PortNum){
	GPIO_TypeDef* PortAdd = NULL;
	switch(PortNum){
	case GPIO_PORTA:
		PortAdd=GPIOA;
		break;
	case GPIO_PORTB:
		PortAdd=GPIOB;
		break;
	case GPIO_PORTC:
		PortAdd=GPIOC;
		break;
	case GPIO_PORTD:
		PortAdd=GPIOD;
		break;
	case GPIO_PORTE:
		PortAdd=GPIOE;
		break;
	default:
		break;
	}
	return PortAdd;
}
//=============================================================================================
