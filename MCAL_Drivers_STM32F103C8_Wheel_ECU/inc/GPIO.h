 /******************************************************************************
 * Module: GPIO
 * File Name: GPIO.h
 * Description: Header file for GPIO driver for stm32f103x6
 * Author: Ehab Mohamed Abdelhamed
 ******************************************************************************/
#ifndef INC_GPIO_H_
#define INC_GPIO_H_

/*******************************************************************************
 *                             Includes			                            	*
 *******************************************************************************/
#include "Common_Macros.h"
#include "Platform_Types.h"

/*******************************************************************************
 *                             	GPIO Data Types                             	*
 *******************************************************************************/

//@REF : GPIO_Modes
/*Supported mode in GPIO
1)Input modes
 Analog mode
 Floating input (reset state)
 Alternate function input
 Input with pull-up / pull-down
2) Output modes
 General purpose output push-pull
 General purpose output Open-drain
 Alternate function output Push-pull
 Alternate function output Open-drain
*/

typedef enum{
	GPIO_INPUT_ANALOG,GPIO_INUPUT_FLOATING,GPIO_AF_INPUT=1,
	GPIO_INPUT_PU,GPIO_INPUT_PD,
	GPIO_OUTPUT_PP,GPIO_OUTPUT_OD,
	GPIO_AF_OUTPUT_PP,GPIO_AF_OUTPUT_OD
}GPIO_Mode_t;


//@REF : GPIO_Speed
/*Supported speed in GPIO
	input default (for input mode)
	Output mode, max speed 10 MHz.
	Output mode, max speed 2 MHz.
	Output mode, max speed 50 MHz*/

typedef enum{
	GPIO_SPEED_INPUT,GPIO_SPEED_10_MHZ,GPIO_SPEED_2_MHZ,GPIO_SPEED_50_MHZ
}GPIO_Speed_t;


typedef struct{
	uint8 Pin_Num;				//Specifies GPIO pin to be configured
								//This parameter can be a value of @REF: GPIO_PINS_NUMBER
	GPIO_Mode_t Pin_Mode;		//Specifies Operation mode of GPIO pin
								//This parameter can be a value of @REF: GPIO_Modes
	GPIO_Speed_t Pin_Speed;		//Specifies Speed of GPIO pin
								//This parameter can be a value of @REF: GPIO_Speed
}GPIO_PinConfig_t;

typedef struct{
	uint8 Pin_Num;				//Specifies GPIO pin to be configured
								//This parameter can be a value of @REF: GPIO_Speed
	uint8 Offest;				//To select the position of start bit from LSB
	uint8 mask;					//to select the number of bits in group
}GPIO_GroupConfig_t;

/*******************************************************************************
 *                             	GPIO Definition                          		*
 *******************************************************************************/
//@REF: GPIO_PINS_NUMBER
#define GPIO_PIN_0			(0U)
#define GPIO_PIN_1			(1U)
#define GPIO_PIN_2			(2U)
#define GPIO_PIN_3			(3U)
#define GPIO_PIN_4			(4U)
#define GPIO_PIN_5			(5U)
#define GPIO_PIN_6			(6U)
#define GPIO_PIN_7			(7U)
#define GPIO_PIN_8			(8U)
#define GPIO_PIN_9			(9U)
#define GPIO_PIN_10			(10U)
#define GPIO_PIN_11			(11U)
#define GPIO_PIN_12			(12U)
#define GPIO_PIN_13			(13U)
#define GPIO_PIN_14			(14U)
#define GPIO_PIN_15			(15U)


//@REF: GPIO_NUMBER
#define GPIO_PORTA			(0U)
#define GPIO_PORTB			(1U)
#define GPIO_PORTC			(2U)
#define GPIO_PORTD			(3U)
#define GPIO_PORTE			(4U)
//@REF:GPIO_PINS_NUMBER
#define PIN_HIGH			(1U)
#define PIN_LOW				(0U)

//@REF:GPIO_LOCKING_STATE
#define LOCKING_ERROR		(0U)
#define LOCKING_SUCCESS		(1U)

//Other Definition
#define INPUT_MODES		 	(2U)
#define OUTPUT_MODES_SHIFT 	(4U)
#define MODE_SHIFT			(4U)
#define CNF_SHIFT			(2U)


/*******************************************************************************
 *					APIs Supported by "MCAL GPIO DRIVER"                       *
 *******************************************************************************/


void MCAL_GPIO_init(uint8 GPIOx_Num,GPIO_PinConfig_t *pinConfig);
void MCAL_GPIO_deInit(uint8 GPIOx_Num);

uint8 MCAL_GPIO_readPin(uint8 GPIOx_Num,uint8 Pin_Num);
uint16 MCAL_GPIO_readPort(uint8 GPIOx_Num);

void MCAL_GPIO_writePin(uint8 GPIOx_Num,uint8 Pin_Num,uint8 Pin_Value);
void MCAL_GPIO_writePort(uint8 GPIOx_Num,uint16 Port_Value);

void MCAL_GPIO_writeGroub(uint8 GPIO_Portx,uint16 mask,uint8 offsets,uint16 value);
uint16 MCAL_GPIO_readGroup(uint8 GPIO_Portx,uint16 mask,uint8 offset);

void MCAL_GPIO_togglePin(uint8 GPIOx_Num,uint8 Pin_Num);

boolean MCAL_GPIO_lockPin(uint8 GPIOx_Num,uint8 Pin_Num);

#endif /* INC_GPIO_H_ */
