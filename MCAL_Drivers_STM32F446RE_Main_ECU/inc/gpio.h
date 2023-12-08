 /******************************************************************************
 * Module: GPIO
 * File Name: gpio.h
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
These bits are written by software to configure the I/O direction mode.
00: Input (reset state)
01: General purpose output mode
10: Alternate function mode
11: Analog mode*/

typedef enum{
	GPIO_INUPUT_Mode,
	GPIO_OUTPUT_PP_Mode,
	GPIO_Alternate_Function_mode,
	Analog_Mode,
	GPIO_OUTPUT_OD_Mode=5,
}GPIO_Mode_t;


//@REF : GPIO_Speed
/*
 * OSPEEDRy[1:0]: Port x configuration bits (y = 0..15)
These bits are written by software to configure the I/O output speed.
00: Low speed
01: Medium speed
10: Fast speed
11: High speed
*/

typedef enum{
	GPIO_SPEED_LOW,GPIO_SPEED_MEDIUM,GPIO_SPEED_FAST,GPIO_SPEED_HIGH
}GPIO_Speed_t;

/*
These bits are written by software to configure the I/O pull-up or pull-down
00: No pull-up, pull-down
01: Pull-up
10: Pull-down
11: Reserved
*/

typedef enum{
	GPIO_NO_PULLUP_POLLDOWN,GPIO_PULL_UP,GPIO_PULL_DOWN
}GPIO_PUD_t;


typedef struct{
	uint8 Pin_Num;				//Specifies GPIO pin to be configured
								//This parameter can be a value of @REF: GPIO_PINS_NUMBER
	GPIO_Mode_t Pin_Mode;		//Specifies Operation mode of GPIO pin
								//This parameter can be a value of @REF: GPIO_Modes
	GPIO_PUD_t Pin_PUD;			//configure the I/O pull-up or pull-down of GPIO_PIN
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
#define GPIO_PORTF			(5U)
#define GPIO_PORTG			(6U)
#define GPIO_PORTH			(7U)

//@REF: AF_NUMBER
#define AF0 	(0U)
#define AF1 	(1U)
#define AF2 	(2U)
#define AF3 	(3U)
#define AF4 	(4U)
#define AF5 	(5U)
#define AF6 	(6U)
#define AF7 	(7U)
#define AF8 	(8U)
#define AF9 	(9U)
#define AF10 	(10U)
#define AF11 	(11U)
#define AF12 	(12U)
#define AF13 	(13U)
#define AF14 	(14U)
#define AF15 	(15U)


//@REF:GPIO_PINS_NUMBER
#define PIN_HIGH			(1U)
#define PIN_LOW				(0U)

//@REF:GPIO_LOCKING_STATE
#define LOCKING_ERROR		(0U)
#define LOCKING_SUCCESS		(1U)



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
