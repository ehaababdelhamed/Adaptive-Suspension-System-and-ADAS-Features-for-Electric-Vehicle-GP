 /******************************************************************************
 * Module: EXTI
 * File Name: EXTI.h
 * Description: Header file for EXTI driver for stm32f103x6
 * Author: Ehab Mohamed Abdelhamed
 ******************************************************************************/

#ifndef INC_EXTI_H_
#define INC_EXTI_H_
/*******************************************************************************
 *                             Includes			                            	*
 *******************************************************************************/
#include "Platform_Types.h"
#include "Common_Macros.h"


/*******************************************************************************
 *                             	EXTI Data Types                             	*
 *******************************************************************************/
typedef enum{
	EXTI0,EXTI1,EXTI2,EXTI3,
	EXTI4,EXTI5,EXTI6,EXTI7,
	EXTI8,EXTI9,EXTI10,EXTI11,
	EXTI12,EXTI13,EXTI14,EXTI15
}EXTI_NUM_e;

//@REF enumeration:EXTI_EDGE_e
typedef enum{
	RAISING_EDGE,FAILING_EGDE,BOTH_EDGES
}EXTI_EDGE_e;

typedef struct{
	EXTI_NUM_e EXTI_Num;
	uint8 GPIO_Port;
	uint8 GPIO_Pin;
	uint16 IRQ_Num;
}EXTI_GPIOMaping;

typedef struct{
	EXTI_GPIOMaping EXTI_Pin;	//Specifies the external interrupt GPIO mapping
								//This Parameter must be based on @REF EXTI_Define
	EXTI_EDGE_e		Edge;		//Specifies the edge on the pin for interrupt
								//This Parameter must be based on @REF enumeration:EXTI_EDGE_e
}EXTI_Config_t;
/*******************************************************************************
 *                             	GPIO Definition                          		*
 *******************************************************************************/
//@REF EXTI_Define
#define EXTI0PA0	(EXTI_GPIOMaping){EXTI0 , GPIO_PORTA , GPIO_PIN_0,EXTI0_IRQ}
#define EXTI0PB0	(EXTI_GPIOMaping){EXTI0 , GPIO_PORTB , GPIO_PIN_0,EXTI0_IRQ}
#define EXTI0PC0	(EXTI_GPIOMaping){EXTI0 , GPIO_PORTC , GPIO_PIN_0,EXTI0_IRQ}
#define EXTI0PD0	(EXTI_GPIOMaping){EXTI0 , GPIO_PORTD , GPIO_PIN_0,EXTI0_IRQ}

#define EXTI1PA1	(EXTI_GPIOMaping){EXTI1 , GPIO_PORTA , GPIO_PIN_1,EXTI1_IRQ}
#define EXTI1PB1	(EXTI_GPIOMaping){EXTI1 , GPIO_PORTB , GPIO_PIN_1,EXTI1_IRQ}
#define EXTI1PC1	(EXTI_GPIOMaping){EXTI1 , GPIO_PORTC , GPIO_PIN_1,EXTI1_IRQ}
#define EXTI1PD1	(EXTI_GPIOMaping){EXTI1 , GPIO_PORTD , GPIO_PIN_1,EXTI1_IRQ}

#define EXTI2PA2	(EXTI_GPIOMaping){EXTI2 , GPIO_PORTA , GPIO_PIN_2,EXTI2_IRQ}
#define EXTI2PB2	(EXTI_GPIOMaping){EXTI2 , GPIO_PORTB , GPIO_PIN_2,EXTI2_IRQ}
#define EXTI2PC2	(EXTI_GPIOMaping){EXTI2 , GPIO_PORTC , GPIO_PIN_2,EXTI2_IRQ}
#define EXTI2PD2	(EXTI_GPIOMaping){EXTI2 , GPIO_PORTD , GPIO_PIN_2,EXTI2_IRQ}

#define EXTI3PA3	(EXTI_GPIOMaping){EXTI3 , GPIO_PORTA , GPIO_PIN_3,EXTI3_IRQ}
#define EXTI3PB3	(EXTI_GPIOMaping){EXTI3 , GPIO_PORTB , GPIO_PIN_3,EXTI3_IRQ}
#define EXTI3PC3	(EXTI_GPIOMaping){EXTI3 , GPIO_PORTC , GPIO_PIN_3,EXTI3_IRQ}
#define EXTI3PD3	(EXTI_GPIOMaping){EXTI3 , GPIO_PORTD , GPIO_PIN_3,EXTI3_IRQ}

#define EXTI4PA4	(EXTI_GPIOMaping){EXTI4 , GPIO_PORTA , GPIO_PIN_4,EXTI4_IRQ}
#define EXTI4PB4	(EXTI_GPIOMaping){EXTI4 , GPIO_PORTB , GPIO_PIN_4,EXTI4_IRQ}
#define EXTI4PC4	(EXTI_GPIOMaping){EXTI4 , GPIO_PORTC , GPIO_PIN_4,EXTI4_IRQ}
#define EXTI4PD4	(EXTI_GPIOMaping){EXTI4 , GPIO_PORTD , GPIO_PIN_4,EXTI4_IRQ}

#define EXTI5PA5	(EXTI_GPIOMaping){EXTI5 , GPIO_PORTA , GPIO_PIN_5,EXTI5_IRQ}
#define EXTI5PB5	(EXTI_GPIOMaping){EXTI5 , GPIO_PORTB , GPIO_PIN_5,EXTI5_IRQ}
#define EXTI5PC5	(EXTI_GPIOMaping){EXTI5 , GPIO_PORTC , GPIO_PIN_5,EXTI5_IRQ}
#define EXTI5PD5	(EXTI_GPIOMaping){EXTI5 , GPIO_PORTD , GPIO_PIN_5,EXTI5_IRQ}

#define EXTI6PA6	(EXTI_GPIOMaping){EXTI6 , GPIO_PORTA , GPIO_PIN_6,EXTI6_IRQ}
#define EXTI6PB6	(EXTI_GPIOMaping){EXTI6 , GPIO_PORTB , GPIO_PIN_6,EXTI6_IRQ}
#define EXTI6PC6	(EXTI_GPIOMaping){EXTI6 , GPIO_PORTC , GPIO_PIN_6,EXTI6_IRQ}
#define EXTI6PD6	(EXTI_GPIOMaping){EXTI6 , GPIO_PORTD , GPIO_PIN_6,EXTI6_IRQ}

#define EXTI7PA7	(EXTI_GPIOMaping){EXTI7 , GPIO_PORTA , GPIO_PIN_7,EXTI7_IRQ}
#define EXTI7PB7	(EXTI_GPIOMaping){EXTI7 , GPIO_PORTB , GPIO_PIN_7,EXTI7_IRQ}
#define EXTI7PC7	(EXTI_GPIOMaping){EXTI7 , GPIO_PORTC , GPIO_PIN_7,EXTI7_IRQ}
#define EXTI7PD7	(EXTI_GPIOMaping){EXTI7 , GPIO_PORTD , GPIO_PIN_7,EXTI7_IRQ}

#define EXTI8PA8	(EXTI_GPIOMaping){EXTI8 , GPIO_PORTA , GPIO_PIN_8,EXTI8_IRQ}
#define EXTI8PB8	(EXTI_GPIOMaping){EXTI8 , GPIO_PORTB , GPIO_PIN_8,EXTI8_IRQ}
#define EXTI8PC8	(EXTI_GPIOMaping){EXTI8 , GPIO_PORTC , GPIO_PIN_8,EXTI8_IRQ}
#define EXTI8PD8	(EXTI_GPIOMaping){EXTI8 , GPIO_PORTD , GPIO_PIN_8,EXTI8_IRQ}

#define EXTI9PA9	(EXTI_GPIOMaping){EXTI9 , GPIO_PORTA , GPIO_PIN_9,EXTI9_IRQ}
#define EXTI9PB9	(EXTI_GPIOMaping){EXTI9 , GPIO_PORTB , GPIO_PIN_9,EXTI9_IRQ}
#define EXTI9PC9	(EXTI_GPIOMaping){EXTI9 , GPIO_PORTC , GPIO_PIN_9,EXTI9_IRQ}
#define EXTI9PD9	(EXTI_GPIOMaping){EXTI9 , GPIO_PORTD , GPIO_PIN_9,EXTI9_IRQ}

#define EXTI10PA10	(EXTI_GPIOMaping){EXTI10 , GPIO_PORTA , GPIO_PIN_10,EXTI10_IRQ}
#define EXTI10PB10	(EXTI_GPIOMaping){EXTI10 , GPIO_PORTB , GPIO_PIN_10,EXTI10_IRQ}
#define EXTI10PC10	(EXTI_GPIOMaping){EXTI10 , GPIO_PORTC , GPIO_PIN_10,EXTI10_IRQ}
#define EXTI10PD10	(EXTI_GPIOMaping){EXTI10 , GPIO_PORTD , GPIO_PIN_10,EXTI10_IRQ}

#define EXTI11PA11	(EXTI_GPIOMaping){EXTI11 , GPIO_PORTA , GPIO_PIN_11,EXTI11_IRQ}
#define EXTI11PB11	(EXTI_GPIOMaping){EXTI11 , GPIO_PORTB , GPIO_PIN_11,EXTI11_IRQ}
#define EXTI11PC11	(EXTI_GPIOMaping){EXTI11 , GPIO_PORTC , GPIO_PIN_11,EXTI11_IRQ}
#define EXTI11PD11	(EXTI_GPIOMaping){EXTI11 , GPIO_PORTD , GPIO_PIN_11,EXTI11_IRQ}

#define EXTI12PA12	(EXTI_GPIOMaping){EXTI12 , GPIO_PORTA , GPIO_PIN_12,EXTI12_IRQ}
#define EXTI12PB12	(EXTI_GPIOMaping){EXTI12 , GPIO_PORTB , GPIO_PIN_12,EXTI12_IRQ}
#define EXTI12PC12	(EXTI_GPIOMaping){EXTI12 , GPIO_PORTC , GPIO_PIN_12,EXTI12_IRQ}
#define EXTI12PD12	(EXTI_GPIOMaping){EXTI12 , GPIO_PORTD , GPIO_PIN_12,EXTI12_IRQ}

#define EXTI13PA13	(EXTI_GPIOMaping){EXTI13 , GPIO_PORTA , GPIO_PIN_13,EXTI13_IRQ}
#define EXTI13PB13	(EXTI_GPIOMaping){EXTI13 , GPIO_PORTB , GPIO_PIN_13,EXTI13_IRQ}
#define EXTI13PC13	(EXTI_GPIOMaping){EXTI13 , GPIO_PORTC , GPIO_PIN_13,EXTI13_IRQ}
#define EXTI13PD13	(EXTI_GPIOMaping){EXTI13 , GPIO_PORTD , GPIO_PIN_13,EXTI13_IRQ}

#define EXTI14PA14	(EXTI_GPIOMaping){EXTI14 , GPIO_PORTA , GPIO_PIN_14,EXTI14_IRQ}
#define EXTI14PB14	(EXTI_GPIOMaping){EXTI14 , GPIO_PORTB , GPIO_PIN_14,EXTI14_IRQ}
#define EXTI14PC14	(EXTI_GPIOMaping){EXTI14 , GPIO_PORTC , GPIO_PIN_14,EXTI14_IRQ}
#define EXTI14PD14	(EXTI_GPIOMaping){EXTI14 , GPIO_PORTD , GPIO_PIN_14,EXTI14_IRQ}

#define EXTI15PA15	(EXTI_GPIOMaping){EXTI15 , GPIO_PORTA , GPIO_PIN_15,EXTI15_IRQ}
#define EXTI15PB15	(EXTI_GPIOMaping){EXTI15 , GPIO_PORTB , GPIO_PIN_15,EXTI15_IRQ}
#define EXTI15PC15	(EXTI_GPIOMaping){EXTI15 , GPIO_PORTC , GPIO_PIN_15,EXTI15_IRQ}
#define EXTI15PD15	(EXTI_GPIOMaping){EXTI15 , GPIO_PORTD , GPIO_PIN_15,EXTI15_IRQ}
//================================================================================
/*******************************************************************************
 *					APIs Supported by "MCAL EXTI DRIVER"                       *
 *******************************************************************************/

void MCAL_EXTI_init(EXTI_Config_t* EXTIConfig);
void MCAL_EXTI_deInit();
void MCAL_EXTI_enable(EXTI_NUM_e EXTI_Number);
void MCAL_EXTI_disable(EXTI_NUM_e EXTI_Number);
void MCAL_EXTI_update(EXTI_Config_t* EXTIConfig);
void MCAL_EXTI_setCallBackFun(EXTI_NUM_e EXTI_Number,void (*callBackFunc)(void));



#endif /* INC_EXTI_H_ */
