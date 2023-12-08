/******************************************************************************
 * Module: SPI
 * File Name: SPI.h
 * Description: Header file for SPI driver for Stm32f103x6
 * Author: Ehab Mohamed Abdelhamed
 ******************************************************************************/
#ifndef INC_SPI_H_
#define INC_SPI_H_
/*******************************************************************************
 *                             Includes			                            	*
 *******************************************************************************/
#include "Common_Macros.h"
#include "Platform_Types.h"

/*******************************************************************************
 *                             	SPI Data Types                             	*
 *******************************************************************************/
//@REF : SPI_Mode_e
typedef enum{						//15:BIDIMODE	10:RXONLY	// M 		S		M		S
	SPI_MODE_FULL_DUPLEX =0, 		//		0			0		//MOSI >> MOSI  & MISO >> MISO
	SPI_MODE_HALF_DUPLEX=2,			//		1			0		//MOSI >> MISO
	SPI_MODE_SIMPLEX_RX=1,			//		0			1		//MISO >> MISO
	SPI_MODE_SIMPLEX_TX = 4			//		0			0		//MOSI >> MOSI
}SPI_Mode_e;

//@REF : SPI_DeviceMode_e
typedef enum{
	SPI_SLAVE,
	SPI_MASTER
}SPI_DeviceMode_e;

//@REF : SPI_PreScaler_e
typedef enum{
	SPI_PERSCALER_2,
	SPI_PERSCALER_4,
	SPI_PERSCALER_8,
	SPI_PERSCALER_16,
	SPI_PERSCALER_32,
	SPI_PERSCALER_64,
	SPI_PERSCALER_128,
	SPI_PERSCALER_256,
}SPI_PreScaler_e;

//@REF : SPI_NSS_e
typedef enum{									//CR1>8:SSI		CR1>9:SSM	CR2>2:SSOE
	SPI_NSS_HW_SLAVE,							//	0				0			0
	SPI_NSS_HW_MultiMaster_OUPTUT_DISABLE=0,	//	0				0			0
	SPI_NSS_HW_MultiMaster_OUPTUT_ENABLE,		//	0				0			1
	SPI_NSS_SW_RESET,							//	0				1			0
	SPI_NSS_SW_SET=6							//	1				1			0
}SPI_NSSMode_e;

//@REF : SPI_CLKPloarity_e
typedef enum{
	SPI_CLKP_IDLE_LOW,
	SPI_CLKP_IDLE_HIGH
}SPI_CLKPloarity_e;

//@REF : SPI_CLKPhase_e
typedef enum{
	SPI_CLKPh_FIRST_EDGE,
	SPI_CLKPh_SECOND_EDGE
}SPI_CLKPhase_e;

//@REF : SPI_DataOrder_e;
typedef enum{
	SPI_DATA_ORDER_MSB_FIRST,
	SPI_DATA_ORDER_LSB_FIRST
}SPI_DataOrder_e;

//@REF : SPI_DataSize_e;
typedef enum{
	SPI_DATA_SIZE_8B,
	SPI_DATA_SIZE_16B
}SPI_DataSize_e;

//@REF : SPI_DataSize_e;
typedef enum{
	SPI_POLLING_DISABLE,
	SPI_POLLING_ENABLE
}SPI_PollingMechanism_e;


typedef struct{
	SPI_Mode_e 			SPI_Mode;			//Specifics SPI Communication mode @REF : SPI_Mode_e
	SPI_DeviceMode_e 	Device_Mode;		//Specifics SPI Device mode(Master/Slave) @REF : SPI_Mode_e
	SPI_PreScaler_e		SPI_Prescaler;		//Specifics SPI BaudRate prescaler  @REF : SPI_Mode_e
											//This parameter is prescaler for APBx Clock
											//So User should configure APBx Clock form RCC before select prescaler to reach to required clock

	SPI_DataSize_e		Data_Size;			//Specifics SPI data frame format 8- or 16-bit @REF SPI_DataSize_e
	SPI_DataOrder_e		Data_Order;			//Specifics SPI  data order (MSB-first or LSB-first) @REF SPI_DataOrder_e
	SPI_CLKPloarity_e	Clock_Polarity;		//Specifics SPI idle level of clock line
	SPI_CLKPhase_e		Clock_Phase;		//Specifics SPI data capture edge
	SPI_NSSMode_e		NSS_Mode;			//Specifics NSS Pin Control by HW or by SW
											/* Software NSS management
											 * he slave select information is driven internally by the value of the SSI bit
											 * Hardware NSS management
											 * For Master: NSS signal is driven low when the master starts the communication and is kept
											 * low until the SPI is disabled
											 * For Salve : the NSS pin acts as a classical NSS input: the
											 * slave is selected when NSS is low and deselected when NSS high.
											 */
}SPI_Config_t;

/*******************************************************************************
 *                             	SPI Definition                             	*
 *******************************************************************************/
#define NUM_OF_SPI_INSTANCES 	(2U)
//@REF : SPI_ID_define
#define	SPI1_ID					(0U)
#define SPI2_ID					(1U)

#define NUM_OF_SPI_IRQ 			(3U)
//@REF : SPI_IRQ_Source_define
#define SPI_IRQ_Source_TXE		(7U)	//TX buffer empty interrupt
#define SPI_IRQ_Source_RXNE		(6U)	//RX buffer not empty interrupt
#define SPI_IRQ_Source_ERR		(5U)	//Error interrupt

#define IRQ_SHIFT_INDEX			(5U)

#define Source_TXE				(2U)
#define Source_RXNE				(1U)
#define Source_ERR				(0U)

/*******************************************************************************
 *					APIs Supported by "MCAL SPI DRIVER" 	                   *
 *******************************************************************************/
void MCAL_SPI_init(uint8 SPIx_ID,const SPI_Config_t *SPIConfig);
void MCAL_SPI_deInit(uint8 SPIx_ID);

void MCAL_SPI_sendData(uint8 SPIx_ID,uint16 TxBuffer,SPI_PollingMechanism_e Polling);
uint16 MCAL_SPI_receiveData(uint8 SPIx_ID,SPI_PollingMechanism_e Polling);
uint16 MCAL_SPI_TX_RX(uint8 SPIx_ID,uint16 TxBuffer,SPI_PollingMechanism_e Polling);

void MCAL_SPI_GPIO_SET_PINS(uint8 SPIx_ID);

void MCAL_SPI_enableInterrupt(uint8 SPIx_ID,uint8 IRQ_Source);
void MCAL_SPI_disableInterrupt(uint8 SPIx_ID,uint8 IRQ_Source);
void MCAL_SPI_setCallBackFunc(uint8 SPIx_ID,uint8 IRQ_Source,void (*callBackFunc)());

void MCAL_SPI_enable(uint8 SPIx_ID);
void MCAL_SPI_disable(uint8 SPIx_ID);


#endif /* INC_SPI_H_ */
