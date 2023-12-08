/******************************************************************************
 * Module: I2C
 * File Name: I2C.h
 * Description: Header file for I2C driver for Stm32f103x6
 * Author: Ehab Mohamed Abdelhamed
 ******************************************************************************/
#ifndef INC_I2C_H_
#define INC_I2C_H_
/*******************************************************************************
 *                             Includes			                            	*
 *******************************************************************************/
#include "Common_Macros.h"
#include "Platform_Types.h"

/*******************************************************************************
 *                             	I2C Data Types                             	*
 *******************************************************************************/

//@REF I2C_Mode
//Note : SMBUS not supported yet
typedef enum{
	I2C_MODE,SMBUS_MODE
}I2C_Mode_e;

//@REF I2C_ClockMode_e
//Note :I2C_FAST_MODE not supported yet
typedef enum{
	I2C_Standard_MODE,I2C_FAST_MODE
}I2C_ClockMode_e;

//@REF I2C_ClockSpeed_e
//Note :I2C_FAST_MODE not supported yet
typedef enum{
	I2C_NM_50KHZ =( 50000U),
	I2C_NM_100KHZ=(100000U),
	I2C_FM_150KHZ=(150000U),
	I2C_FM_200KHZ=(200000U),
	I2C_FM_250KHZ=(250000U),
	I2C_FM_300KHZ=(300000U),
	I2C_FM_350KHZ=(350000U),
	I2C_FM_400KHZ=(400000U)
}I2C_ClockSpeed_e;


//@REF I2C_StretchingMode_e
//Note :I2C_FAST_MODE not supported yet
typedef enum{
	I2C_Stretching_Enable,I2C_Stretching_Disable
}I2C_StretchingMode_e;

//@REF I2C_DualAddress_e
typedef enum{
	I2C_DualAddress_Disable,I2C_DualAddress_Enable
}I2C_DualAddress_e;

//@REF I2C_ACK_e
typedef enum{
	I2C_ACK_Disable,I2C_ACK_Enable
}I2C_ACK_e;


//@REF I2C_GeneralCall_e
typedef enum{
	I2C_GeneralCall_Disable,I2C_GeneralCall_Enable
}I2C_GeneralCall_e;


//@REF I2C_SlaveAddressMode_e
typedef enum{
	I2C_Slave_Address_7_Bit,I2C_Slave_Address_10_Bit
}I2C_SlaveAddressMode_e;

typedef struct{
	I2C_DualAddress_e I2C_Dual_Address_State;
	I2C_SlaveAddressMode_e I2C_Slave_Address_Mode;
	uint16 I2C_Primary_Slave_Address;
	uint8 I2C_Secondary_Slave_Address;
}I2C_SlaveAddressConfig;

typedef struct{
	I2C_DualAddress_e I2C_Dual_Address_State;
}I2C_IRQConfig;



/*Primary address
 * Secondary address*/
typedef struct{
	I2C_Mode_e I2C_Mode;							//Specifics I2C Bus mode @REF : SPI_Mode
	I2C_ClockMode_e I2C_Clock_Mode;					//Specifics I2C Clock mode(NM/FM) @REF: I2C_ClockMode_e
	I2C_ClockSpeed_e I2C_Clock_Speed;				//Specifics I2C Clock speed @REF: I2C_ClockSpeed_e
	I2C_ACK_e I2C_ACK_state;						//Specifics I2C ACK state @REF : I2C_ACK_e
	I2C_ACK_e I2C_GeneralCall_state;				//Specifics I2C General Call state @REF : I2C_ACK_e
	I2C_StretchingMode_e I2C_Stretching_state;		//Specifics I2C stretching state @REF : I2C_StretchingMode_e
	I2C_SlaveAddressConfig Slave_Address_Config;	//Specifics I2C Slave Address Configuration @REF : SPI_Mode
}I2C_Config_t;


typedef enum{
	START_Repeated,START_GEN
}Repeated_Start_e;

typedef enum{
	Stop_Send,Stop_Not_Send
}Stop_Condition_e;

typedef enum{
	EV5_SB,				//EV5:	bit0: SB=1, cleared by reading SR1 register followed by writing DR register with Address.
	EV1_ADDR,			//EV1: 	bit1 :ADDR=1, cleared by reading SR1 register followed by reading SR2.
	EV6_ADDR=1,			//EV6: 	bit1 :ADDR=1, cleared by reading SR1 register followed by reading SR2.
	EV8_2_BTF,			//EV8_2:bit2 BTF = 1 and TXE = 1, Program Stop request. TxE and BTF are cleared by hardware by the Stop condition
	EV4_STOPF=4,		//EV4: Bit 4 >> SR1 STOPF (Stop detection)
	EV7_RXNE=6,			//EV7: 	bit6 RxNE=1 data register Not empty, cleared by writing DR register
	EV2_RXNE=6,			//EV2: 	bit6 RxNE=1 data register Not empty, cleared by writing DR register
	EV3_TXE=7,			//EV3: 	bit7 TxE=1 data register empty, cleared by writing DR register
	EV8_TXE=7,			//EV8: 	bit7 TxE=1 data register empty, cleared by writing DR register
	EV3_2_AF =10,		//EV3_2 bit10 AF;
	BUSY_FlAG,			//Bit 1 >> SR2
	MSL_TRA_BUSY_TXE=(uint32)(0x00070080) 	//MSL,BUSY,TRA,TXE Bits[0,1,2] SR2 & bit[7] SR1
}I2C_Flags_e;

typedef enum{
	Flag_Disbale ,
	Flag_Enable
}Flag_State_e;

typedef enum{
	I2C_WRITE,
	I2C_READ
}I2C_Directio_e;

typedef enum{
	I2C_POLLING_DISABLE,
	I2C_POLLING_ENABLE
}I2C_PollingMechanism_e;


typedef enum{
	I2C_IRQ_EVENT_WITHOUT_TX_RX,
	I2C_IRQ_ALL_EVENT,
	I2C_IRQ_ALL_ERROR
}I2C_IRQ_e;


typedef enum{
	I2C_EV_SB_IRQ,  		//Start bit sent (Master)
	I2C_EV_ADDR_IRQ,		//Address sent (Master) or Address matched (Slave
	I2C_EV_ADDR10_IRQ,		//10-bit header sent (Master)
	I2C_EV_STOPF_IRQ,		//Stop received (Slave)
	I2C_EV_BTF_IRQ,			//Data byte transfer finished
	I2C_EV_RxNE_IRQ,		//Receive buffer not empty
	I2C_EV_TxE_IRQ,			//Transmit buffer empty
}I2C_IRQ_Source_e;


/*******************************************************************************
 *                             	I2C Definition                             	*
 *******************************************************************************/
#define NUM_OF_I2C_INSTANCES 	(2U)
//@REF : I2C_ID_define
#define	I2C1_ID					(0U)
#define I2C2_ID					(1U)

/*******************************************************************************
 *					APIs Supported by "MCAL I2C DRIVER" 	                   *
 *******************************************************************************/
void MCAL_I2C_init(uint8 I2Cx_ID,I2C_Config_t *I2Cx_Config);
void MCAL_I2C_deinit(uint8 I2Cx_ID);
void MCAL_I2C_GPIO_Set_Pins(uint8 I2Cx_ID);
void MCAL_I2C_MASTER_TX(uint8 I2Cx_ID,uint16 SlaveAddress,uint8 *TxBuffer,uint8 BufferSize,Stop_Condition_e Stop,Repeated_Start_e Start);
void MCAL_I2C_MASTER_RX(uint8 I2Cx_ID,uint16 SlaveAddress,uint8 *TxBuffer,uint8 BufferSize,Stop_Condition_e Stop,Repeated_Start_e Start);


void I2C_Generate_Start(uint8 I2Cx_ID,Flag_State_e state,Repeated_Start_e Start);
void I2C_Generate_Stop(uint8 I2Cx_ID,Flag_State_e state);
void I2C_SendSlaveAddress(uint8 I2Cx_ID,uint16 SlaveAdd,I2C_Directio_e direction);
Flag_State_e I2C_GetFlagState(uint8 I2Cx_ID,I2C_Flags_e Flag);
#endif /* INC_I2C_H_ */
