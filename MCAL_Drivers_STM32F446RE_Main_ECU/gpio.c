/******************************************************************************
 * Module: GPIO
 * File Name: gpio.c
 * Description: Source file for GPIO driver for stm32f103x6
 * Author: Ehab Mohamed Abdelhamed
 ******************************************************************************/

/*******************************************************************************
 *                             Includes			                            	*
 *******************************************************************************/
#include "stm32f446xx.h"
#include "gpio.h"


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
 * Parameters (in)	: GPIO_PORT_Num- Number of Specific instance of GPIO @REF: GPIO_NUMBER
 * 					  pinConfig - Pointer to pin configuration date
 * Return value		: None
 * Note				: Stm32f103x6 supported GPIOA,GPIOB,GPIOC,GPIOD,GPIOE
 * 					  but LQF48 Package supported fully GPIOA,GPIOB
 * 					  and partially GPIOC[13-15] and GPIOD[0-1]
 ************************************************************************************/

void MCAL_GPIO_init(uint8 GPIO_PORT_Num,GPIO_PinConfig_t *pinConfig){
	boolean error=FALSE;				//to check the if inputs is valid
	GPIO_TypeDef *GPIOx;				//Store Address of GPIOx
	GPIOx=GPIO_GetPortAdd(GPIO_PORT_Num);
	if(GPIOx == NULL ||pinConfig == NULL || pinConfig->Pin_Num < GPIO_PIN_0 || pinConfig->Pin_Num > GPIO_PIN_15){
		error=TRUE;
	}
	if(error == FALSE){

		//1.configure the I/O direction mode
		GPIOx->MODER &= ~(0b11<<(2*pinConfig->Pin_Num));
		GPIOx->MODER |= ((pinConfig->Pin_Mode&0b11)<<(2*pinConfig->Pin_Num));

		//2.configure the I/O output speed.
		GPIOx->OSPEEDER &= ~(0b11<<(2*pinConfig->Pin_Num));
		GPIOx->OSPEEDER |= (pinConfig->Pin_Speed<<(2*pinConfig->Pin_Num));

		//3.  configure the I/O pull-up or pull-down
		GPIOx->PUPDR &= ~(0b11<<(2*pinConfig->Pin_Num));
		GPIOx->PUPDR |= (pinConfig->Pin_PUD<<(2*pinConfig->Pin_Num));

		//4. configure the output type of the I/O port
		if(pinConfig->Pin_Mode == GPIO_OUTPUT_OD_Mode){
			SET_BIT(GPIOx->OTYPER,pinConfig->Pin_Num);
		}else{
			CLEAR_BIT(GPIOx->OTYPER,pinConfig->Pin_Num);
		}
	}
}

/************************************************************************************
 * Function Name	: MCAL_GPIO_deInit
 * Description		: Function to reset all registers of specific GPIO.
 * Parameters (in)	: GPIO_PORT_Num- Number of Specific instance of GPIO @REF: GPIO_NUMBER
 * Return value		: None
 * Note				: None
 ************************************************************************************/

void MCAL_GPIO_deInit(uint8 GPIO_PORT_Num){
	//Assign all registers to reset value
	//Use RCC Module to reset
	if(GPIO_PORT_Num == GPIO_PORTA){
		RCC_GPIOA_RESET();
	}else if(GPIO_PORT_Num == GPIO_PORTB){
		RCC_GPIOB_RESET();
	}else if(GPIO_PORT_Num == GPIO_PORTC){
		RCC_GPIOC_RESET();
	}else if(GPIO_PORT_Num == GPIO_PORTD){
		RCC_GPIOD_RESET();
	}else if(GPIO_PORT_Num == GPIO_PORTE){
		RCC_GPIOE_RESET();
	}else if(GPIO_PORT_Num == GPIO_PORTF){
		RCC_GPIOF_RESET();
	}else if(GPIO_PORT_Num == GPIO_PORTG){
		RCC_GPIOG_RESET();
	}else if(GPIO_PORT_Num == GPIO_PORTH){
		RCC_GPIOH_RESET();
	}
}


/************************************************************************************
 * Function Name	: MCAL_GPIO_readPin
 * Description		: Function to read specific pin of specific GPIO.
 * Parameters (in)	: GPIO_PORT_Num- Number of Specific instance of GPIO @REF: GPIO_NUMBER
 * 					  GPIO_Pin_Num	- Pin number according @REF:GPIO_PINS_NUMBER
 * Return value		: PinLevelType according  @REF:GPIO_PIN_LEVEL
 * Note				: None
 ************************************************************************************/
uint8 MCAL_GPIO_readPin(uint8 GPIO_PORT_Num,uint8 GPIO_Pin_Num){
	uint8 PinLevelType=PIN_LOW;			//To store the pin value
	boolean error=FALSE;				//To check the if inputs is valid
	GPIO_TypeDef *GPIOx;				//Store Address of GPIOx
	GPIOx=GPIO_GetPortAdd(GPIO_PORT_Num);
	if(GPIOx == NULL || GPIO_Pin_Num< GPIO_PIN_0 || GPIO_Pin_Num > GPIO_PIN_15){
		error=TRUE;
	}
	//if input valid >> Read the pin value from GPIO_IDR Register
	if(error == FALSE){
		if(BIT_IS_SET(GPIOx->IDR,GPIO_Pin_Num) == PIN_LOW ){
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
 * Parameters (in)	: GPIO_PORT_Num- Number of Specific instance of GPIO @REF: GPIO_NUMBER
 * Return value		: PortLevelType
 * Note				: None
 ************************************************************************************/
uint16 MCAL_GPIO_readPort(uint8 GPIO_PORT_Num){
	uint8 PortLevelType=0;
	boolean error=FALSE;				//to check the if inputs is valid
	GPIO_TypeDef *GPIOx;				//Store Address of GPIOx
	GPIOx=GPIO_GetPortAdd(GPIO_PORT_Num);
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
 * Parameters (in)	: GPIO_PORT_Num- Number of Specific instance of GPIO @REF: GPIO_NUMBER
 * 					  GPIO_Pin_Num	- Pin number according @REF:GPIO_PINS_NUMBER
 * 					  PIN_Value - Pin value according @REF:GPIO_PIN_LEVEL
 * Return value		: None
 * Note				: None
 ************************************************************************************/
void MCAL_GPIO_writePin(uint8 GPIO_PORT_Num,uint8 GPIO_Pin_Num,uint8 PIN_Value){
	boolean error=FALSE;				//to check the if inputs is valid
	GPIO_TypeDef *GPIOx;				//Store Address of GPIOx
	GPIOx=GPIO_GetPortAdd(GPIO_PORT_Num);
	if(GPIOx == NULL || GPIO_Pin_Num < GPIO_PIN_0 || GPIO_Pin_Num > GPIO_PIN_15){
		error=TRUE;
	}
	//if input valid >> Write the value in the required pin
	if(error == FALSE){
		if(PIN_Value == PIN_HIGH){
			/* Port bit set/reset register (GPIOx_BSRR)
			 * Bits 15:0 BSy: Port x Set bit y (y= 0 .. 15)
			 * These bits are write-only and can be accessed in Word mode only.
			 * 0: No action on the corresponding ODRx bit
			 * 1: Set the corresponding ODRx bit*/
			//GPIOx->BSRR = (1<<GPIO_Pin_Num);
			SET_BIT(GPIOx->ODR,GPIO_Pin_Num);
		}else if(PIN_Value == PIN_LOW){
			/* Port bit reset register (GPIOx_BRR)
			 * Bits 15:0 BRy: Port x Reset bit y (y= 0 .. 15)
			 * These bits are write-only and can be accessed in Word mode only.
			 * 0: No action on the corresponding ODRx bit
			 * 1: Reset the corresponding ODRx bit*/
			//GPIOx->BRR = (1<<GPIO_Pin_Num);
			CLEAR_BIT(GPIOx->ODR,GPIO_Pin_Num);
		}
	}
}

/************************************************************************************
 * Function Name	: MCAL_GPIO_writePort
 * Description		: Function to write in all port of specific GPIO.
 * Parameters (in)	: GPIO_PORT_Num- Number of Specific instance of GPIO @REF: GPIO_NUMBER
 * 					  Port_Value - Port value
 * Return value		: None
 * Note				: None
 ************************************************************************************/
void MCAL_GPIO_writePort(uint8 GPIO_PORT_Num,uint16 Port_Value){
	boolean error=FALSE;				//to check the if inputs is valid
	GPIO_TypeDef *GPIOx;				//Store Address of GPIOx
	GPIOx=GPIO_GetPortAdd(GPIO_PORT_Num);
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
 * Parameters (in)	: GPIO_PORT_Num- Number of Specific instance of GPIO @REF: GPIO_NUMBER
 * 					  Port_Value - Port value
 * 					  offset - To select the position of from LSB
					  mask 	 - To Mask group of bits From start position
 * Return value		: None
 * Note				: None
 ************************************************************************************/
void MCAL_GPIO_writeGroub(uint8 GPIO_PORT_Num,uint16 mask,uint8 offset,uint16 value){
	boolean error=FALSE;				//to check the if inputs is valid
	GPIO_TypeDef *GPIOx;				//Store Address of GPIOx
	GPIOx=GPIO_GetPortAdd(GPIO_PORT_Num);
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
 * Parameters (in)	: GPIO_PORT_Num- Number of Specific instance of GPIO @REF: GPIO_NUMBER
 * 					  Port_Value - Port value
 * 					  offset - To select the position of from LSB
					  mask 	 - To Select Pins to read
 * Return value		: None
 * Note				: None
 ************************************************************************************/
uint16 MCAL_GPIO_readGroup(uint8 GPIO_PORT_Num,uint16 mask,uint8 offset){
	uint8 PortLevelType=0;
	boolean error=FALSE;				//to check the if inputs is valid
	GPIO_TypeDef *GPIOx;				//Store Address of GPIOx
	GPIOx=GPIO_GetPortAdd(GPIO_PORT_Num);
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
 * Parameters (in)	: GPIO_PORT_Num- Number of Specific instance of GPIO @REF: GPIO_NUMBER
 * 					  GPIO_Pin_Num	- Pin number according @REF:GPIO_PINS_NUMBER
 * Return value		: None
 * Note				: None
 ************************************************************************************/
void MCAL_GPIO_togglePin(uint8 GPIO_PORT_Num,uint8 GPIO_Pin_Num){
	boolean error=FALSE;				//to check the if inputs is valid
	GPIO_TypeDef *GPIOx;				//Store Address of GPIOx
	GPIOx=GPIO_GetPortAdd(GPIO_PORT_Num);
	//if input valid >> toggle the required pin
	if(GPIOx == NULL || GPIO_Pin_Num< GPIO_PIN_0 || GPIO_Pin_Num > GPIO_PIN_15){
		error=TRUE;
	}
	if(error == FALSE){
			TOGGLE_BIT(GPIOx->ODR,GPIO_Pin_Num);
	}
}

/************************************************************************************
 * Function Name	: MCAL_GPIO_togglePin
 * Description		: Function to toggle specific pin of specific GPIO.
 * Parameters (in)	: GPIO_PORT_Num- Number of Specific instance of GPIO @REF: GPIO_NUMBER
 * 					  GPIO_Pin_Num	- Pin number according @REF:GPIO_PINS_NUMBER
 * Return value		: None
 * Note				: None
 ************************************************************************************/
void MCAL_GPIO_SetAlternativeFunc(uint8 GPIO_PORT_Num,uint8 GPIO_Pin_Num,uint8 AF_Num){
	boolean error=FALSE;				//to check the if inputs is valid
	GPIO_TypeDef *GPIOx;				//Store Address of GPIOx
	GPIOx=GPIO_GetPortAdd(GPIO_PORT_Num);
	//if input valid >> toggle the required pin
	if(GPIOx == NULL){
		error=TRUE;
	}
	if(error == FALSE){
		if(GPIO_Pin_Num <= GPIO_PIN_7){
			GPIOx->AFRL &= ~(0b1111<<(4*GPIO_Pin_Num));
			GPIOx->AFRL |= (AF_Num<<(4*GPIO_Pin_Num));
		}else{
			GPIOx->AFRH &= ~(0b1111<<(4*(GPIO_Pin_Num-8)));
			GPIOx->AFRH |= (AF_Num<<(4*GPIO_Pin_Num));
		}
	}
}



/************************************************************************************
 * Function Name	: MCAL_GPIO_lockPin
 * Description		: Function to frozen I/O configuration pin
 * Parameters (in)	: GPIO_PORT_Num- Number of Specific instance of GPIO @REF: GPIO_NUMBER
 * 					  GPIO_Pin_Num	- Pin number according @REF:GPIO_PINS_NUMBER
 * Return value		: Locking state (LOCKING_SUCCESS,LOCKING_ERROR)    @REF:GPIO_LOCKING_STATE
 * Note				: None
 ************************************************************************************/
boolean MCAL_GPIO_lockPin(uint8 GPIO_PORT_Num,uint8 GPIO_Pin_Num){
	boolean error=FALSE;				//to check the if inputs is valid
	GPIO_TypeDef *GPIOx;				//Store Address of GPIOx
	uint8 temp;
	boolean lockStatus = LOCKING_ERROR;
	GPIOx=GPIO_GetPortAdd(GPIO_PORT_Num);
	if(GPIOx == NULL || GPIO_Pin_Num< GPIO_PIN_0 || GPIO_Pin_Num > GPIO_PIN_15){
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
		temp =(1<<16) |(1<<GPIO_Pin_Num);
		GPIOx->LCKR = temp; 		//Write 1 >> bit16
		GPIOx->LCKR = (1<<GPIO_Pin_Num); //Write 0 >> bit16
		GPIOx->LCKR = temp; 		//Write 1 >> bit16
		GPIOx->LCKR = (1<<GPIO_Pin_Num); //Write 0 >> bit16
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
static GPIO_TypeDef* GPIO_GetPortAdd(uint8 GPIO_PORT_Num){
	GPIO_TypeDef* PortAdd = NULL;
	switch(GPIO_PORT_Num){
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
	case GPIO_PORTF:
		PortAdd=GPIOF;
		break;
	case GPIO_PORTG:
		PortAdd=GPIOG;
		break;
	case GPIO_PORTH:
		PortAdd=GPIOH;
		break;
	default:
		break;
	}
	return PortAdd;
}
//=============================================================================================
