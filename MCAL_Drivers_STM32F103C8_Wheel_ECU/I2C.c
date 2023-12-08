/******************************************************************************
 * Module: I2C
 * File Name: I2C.c
 * Description: Source file for I2C driver for Stm32f103x6
 * Author: Ehab Mohamed Abdelhamed
 ******************************************************************************/

/*******************************************************************************
 *                             Includes			                            	*
 *******************************************************************************/
#include "stm32f103x6.h"
#include "I2C.h"
#include "RCC.h"
#include "GPIO.h"
/*******************************************************************************
 *								Global Variables	                   			*
 *******************************************************************************/
static I2C_Config_t Global_I2C_Config[NUM_OF_I2C_INSTANCES];
static boolean I2C_initailzation[NUM_OF_I2C_INSTANCES]={FALSE,FALSE};
static void(*GP_CallBackFun[NUM_OF_I2C_INSTANCES])(I2C_IRQ_Source_e Soruce)= {NULL,NULL};


/*******************************************************************************
 *									Macros			                   			*
 *******************************************************************************/

#define GET_I2C_BASE_ADD(I2Cx_ID) ((I2Cx_ID) == I2C1_ID)? (I2C1) : (((I2Cx_ID) == I2C2_ID)? I2C2 : NULL)

/*******************************************************************************
 *                            	Function Definition	                   			*
 *******************************************************************************/
void MCAL_I2C_init(uint8 I2Cx_ID,I2C_Config_t *I2Cx_Config){
	I2C_TypeDef *I2Cx;
	boolean error=FALSE;
	uint32 Fpclk1=0;
	uint8 Fpclk2_Mhz=0;
	uint8 CCR_Value=0;

	I2Cx = GET_I2C_BASE_ADD(I2Cx_ID);

	if(I2Cx == NULL){
		error = TRUE;
	}

	if(error == FALSE){
		//Store Configuration in global variable
		Global_I2C_Config[I2Cx_ID] = (I2C_Config_t)(*I2Cx_Config);
		I2C_initailzation[I2Cx_ID] = TRUE;
		//1. Enable I2C RCC Clock
		if(I2Cx_ID == I2C1_ID){
			RCC_I2C1_CLK_EN();
		}else if(I2Cx_ID == I2C2_ID){
			RCC_I2C2_CLK_EN();
		}

		if(I2Cx_Config->I2C_Mode == I2C_MODE){
			/*
			 * The following is the required sequence in master mode.
			 * Program the peripheral input clock in I2C_CR2 Register in order to generate correct timings.
			• Configure the clock control registers.
			• Configure the rise time register.
			• Program the I2C_CR1 register to enable the peripheral.
			• Set the START bit in the I2C_CR1 register to generate a Start condition.
			 */
			/*=========================== I2C CLock Configuration ===========================*/
			I2Cx->CR1.bit.PE = FALSE; //Disable Peripheral

			if(I2Cx_Config->I2C_Clock_Mode == I2C_Standard_MODE){
				//Get Clock OF APB1 Bus.
				Fpclk1 = MCAL_RCC_GetPCLK1();
				//Calculate frequency in MHZ.
				Fpclk2_Mhz= (uint8)((Fpclk1/1000000));
				I2Cx->CR2.bit.FREQ = (Fpclk2_Mhz & 0x3F);\
				//Set clock mode
				I2Cx->CCR.bit.F_S = I2Cx_Config->I2C_Clock_Mode;
				//Calculate CRR value
				/* Thigh = CRR * Tpclk1
				 * Tpclk1/2 = CRR * Tpclk1
				 * CRR = Frequency of APB2 Clock / 2*Frequency of I2C Clock
				 * */
				CCR_Value = (Fpclk1/(I2Cx_Config->I2C_Clock_Speed<<1));
				I2Cx->CCR.bit.CRR0_11 = CCR_Value;
				//Set Maximum Rise Time
				I2Cx->TRISE = ((Fpclk2_Mhz+1) & 0x3F);
			}else if(I2Cx_Config->I2C_Clock_Mode == I2C_FAST_MODE){
				//Fast Mode Not Supported Yet
			}

			/*=========================== I2C CR1 Configuration ===========================*/
			I2Cx->CR1.bit.SMBUS = I2Cx_Config->I2C_Mode;
			I2Cx->CR1.bit.ACK = I2Cx_Config->I2C_ACK_state;
			I2Cx->CR1.bit.NO_STRETCH = I2Cx_Config->I2C_Stretching_state;
			I2Cx->CR1.bit.ENGC = I2Cx_Config->I2C_GeneralCall_state;

			/*======================== I2C OAR1&OAR2 Configuration =======================*/
			//Set Slave Address Mode
			I2Cx->OAR1.bit.ADD_MODE = (I2Cx_Config->Slave_Address_Config.I2C_Slave_Address_Mode);
			//Bit 14 Should always be kept at 1 by software
			I2Cx->OAR1.bit.BIT14 = TRUE;

			//Set required Slave Address
			if(I2Cx_Config->Slave_Address_Config.I2C_Slave_Address_Mode ==I2C_Slave_Address_7_Bit){
				I2Cx->OAR1.bit.ADD =((I2Cx_Config->Slave_Address_Config.I2C_Primary_Slave_Address &0x7f)<<1);
			}else if(I2Cx_Config->Slave_Address_Config.I2C_Slave_Address_Mode ==I2C_Slave_Address_10_Bit){
				I2Cx->OAR1.bit.ADD =((I2Cx_Config->Slave_Address_Config.I2C_Primary_Slave_Address &0x3ff));
			}

			//Set Dual Slave address is requried
			if(I2Cx_Config->Slave_Address_Config.I2C_Dual_Address_State == I2C_DualAddress_Enable){
				I2Cx->OAR2.bit.ENDUAL= I2C_DualAddress_Enable;
				I2Cx->OAR2.bit.ADD2 =(I2Cx_Config->Slave_Address_Config.I2C_Primary_Slave_Address &0x7f);
			}

		}else if(I2Cx_Config->I2C_Mode == SMBUS_MODE ){
			//SMBUS_MODE Not supported yet
		}

		//Enable I2C Peripheral
		I2Cx->CR1.bit.PE = TRUE;
	}
}




void MCAL_I2C_deinit(uint8 I2Cx_ID){
	I2C_TypeDef *I2Cx;
	boolean error=FALSE;
	I2Cx = GET_I2C_BASE_ADD(I2Cx_ID);
	if(I2Cx == NULL){
		error = TRUE;
	}
	if(error == FALSE){
		if(I2Cx_ID == I2C1_ID){
			NVIC_IRQ31_I2C1_EV_Disable();
			NVIC_IRQ32_I2C1_ER_Disable();
			RCC_I2C1_RESET();
		}else if(I2Cx_ID == I2C2_ID){
			NVIC_IRQ33_I2C2_EV_Disable();
			NVIC_IRQ34_I2C2_ER_Disable();
			RCC_I2C1_RESET();
		}
	}
}


void MCAL_I2C_GPIO_Set_Pins(uint8 I2Cx_ID){
	//I2C1 >> SCL: PB6  , SDA: PB7
	//I2C2 >> SCL: PB10 , SDA: PB11
	GPIO_PinConfig_t Pin_Conifg={GPIO_PIN_6,GPIO_AF_OUTPUT_OD,GPIO_SPEED_10_MHZ};
	if(I2Cx_ID == I2C1_ID){
		Pin_Conifg.Pin_Num = GPIO_PIN_6;
		MCAL_GPIO_init(GPIO_PORTB, &Pin_Conifg);
		Pin_Conifg.Pin_Num = GPIO_PIN_7;
		MCAL_GPIO_init(GPIO_PORTB, &Pin_Conifg);
	}else if(I2Cx_ID == I2C2_ID){
		Pin_Conifg.Pin_Num = GPIO_PIN_10;
		MCAL_GPIO_init(GPIO_PORTB, &Pin_Conifg);
		Pin_Conifg.Pin_Num = GPIO_PIN_11;
		MCAL_GPIO_init(GPIO_PORTB, &Pin_Conifg);
	}
}



void MCAL_I2C_MASTER_TX(uint8 I2Cx_ID,uint16 SlaveAddress,uint8 *TxBuffer,uint8 BufferSize,Stop_Condition_e Stop,Repeated_Start_e Start){
	I2C_TypeDef *I2Cx;
	boolean error=FALSE;
	uint8 count=0;
	I2Cx = GET_I2C_BASE_ADD(I2Cx_ID);
	if(I2Cx == NULL){
		error = TRUE;
	}
	if(error == FALSE){
		//Frame : [S] (EV5) [Slave Address] {A} (EV6 ,EV8-1) [Data] {A} (EV8) ..... (EV8_2) [P]

		//1.set Start bit in I2C_CR1 to generate start condition
		I2C_Generate_Start(I2Cx_ID,Flag_Enable,Start);

		//2.Wait EV5
		//EV5:	SR1_bit0: SB=1, cleared by reading SR1 register followed by writing DR register with Address.
		while(I2C_GetFlagState(I2Cx_ID,EV5_SB) != Flag_Enable);

		//3.Send Slave address with R/W bit
		I2C_SendSlaveAddress(I2Cx_ID,SlaveAddress,I2C_WRITE);

		//4.wait EV6 (address matched)
		//EV6: 	SR1_bit1 :ADDR=1, cleared by reading SR1 register followed by reading SR2
		while(I2C_GetFlagState(I2Cx_ID,EV6_ADDR) != Flag_Enable);

		//5.Check MSL,BUSY,TRA,TXE Flags
		while(I2C_GetFlagState(I2Cx_ID,MSL_TRA_BUSY_TXE) != Flag_Enable);

		//6.Send Data byte by byte followed by waiting TXE flag is set
		for(count=0;count<BufferSize;count++){
			I2Cx->DR = TxBuffer[count];

			//EV8: 	bit7 TxE=1 data register empty, cleared by writing DR register
			while(I2C_GetFlagState(I2Cx_ID,EV8_TXE) != Flag_Enable);
		}
		//Wait data transfer is finished
		while(I2C_GetFlagState(I2Cx_ID,EV8_2_BTF) != Flag_Enable);

		//7.Generate  stop condition
		if(Stop == Stop_Send){
			I2C_Generate_Stop(I2Cx_ID,Flag_Enable);
		}
	}
}



void MCAL_I2C_MASTER_RX(uint8 I2Cx_ID,uint16 SlaveAddress,uint8 *RxBuffer,uint8 BufferSize,Stop_Condition_e Stop,Repeated_Start_e Start){
	I2C_TypeDef *I2Cx;
	boolean error=FALSE;
	uint8 count=0;
	I2Cx = GET_I2C_BASE_ADD(I2Cx_ID);
	if(I2Cx == NULL){
		error = TRUE;
	}
	if(error == FALSE){
		//Frame : [S] (EV5) [Slave Address] {A} (EV6) (EV7){Data} [A]  ..... {DataN}(EV7) [NAK][P]

		//1.set Start bit in I2C_CR1 to generate start condition
		I2C_Generate_Start(I2Cx_ID,Flag_Enable,Start);

		//2.Wait EV5
		//EV5:	SR1_bit0: SB=1, cleared by reading SR1 register followed by writing DR register with Address.
		while(I2C_GetFlagState(I2Cx_ID,EV5_SB) != Flag_Enable);

		//3.Send Slave address with R/W bit
		I2C_SendSlaveAddress(I2Cx_ID,SlaveAddress,I2C_READ);

		//4.wait EV6 (address matched)
		//EV6: 	SR1_bit1 :ADDR=1, cleared by reading SR1 register followed by reading SR2
		while(I2C_GetFlagState(I2Cx_ID,EV6_ADDR) != Flag_Enable);

		//Enable ACK
		I2Cx->CR1.bit.ACK = I2C_ACK_Enable;

		//5.Wait EV7 RXNE flag is set then receive byte
		for(count=0;count<BufferSize;count++){
			//EV7: 	bit6 RxNE=1 data register Not empty, cleared by writing DR register
			while(I2C_GetFlagState(I2Cx_ID,EV7_RXNE) != Flag_Enable);
			RxBuffer[count] = I2Cx->DR;
		}

		//Disable ACK
		I2Cx->CR1.bit.ACK = I2C_ACK_Disable;

		//6.Generate stop condition
		if(Stop == Stop_Send){
			I2C_Generate_Stop(I2Cx_ID,Flag_Enable);
		}

		//Set ACK to default configuration value
		I2Cx->CR1.bit.ACK = Global_I2C_Config[I2Cx_ID].I2C_ACK_state;
	}
}




void MCAL_I2C_Slave_TX(uint8 I2Cx_ID,uint8 data,I2C_PollingMechanism_e Polling){
	I2C_TypeDef *I2Cx;
	boolean error=FALSE;
	I2Cx = GET_I2C_BASE_ADD(I2Cx_ID);
	if(I2Cx == NULL){
		error = TRUE;
	}
	if(error == FALSE){
		if(Polling == I2C_POLLING_ENABLE){
			//wait EV1 (address matched)
			//EV1: 	SR1_bit1 :ADDR=1, cleared by reading SR1 register followed by reading SR2
			while(I2C_GetFlagState(I2Cx_ID,EV1_ADDR) != Flag_Enable);
			//EV3: 	bit7 TxE=1 data register empty, cleared by writing DR register
			while(I2C_GetFlagState(I2Cx_ID,EV3_TXE) != Flag_Enable);
		}
		I2Cx->DR = data;
	}
}



uint8 MCAL_I2C_Slave_RX(uint8 I2Cx_ID,I2C_PollingMechanism_e Polling){
	uint8 data=0;
	I2C_TypeDef *I2Cx;
	boolean error=FALSE;
	I2Cx = GET_I2C_BASE_ADD(I2Cx_ID);
	if(I2Cx == NULL){
		error = TRUE;
	}
	if(error == FALSE){
		if(Polling == I2C_POLLING_ENABLE){

			//Wait EV1 (address matched)
			//EV1: 	SR1_bit1 ADDR=1, cleared by reading SR1 register followed by reading SR1
			while(I2C_GetFlagState(I2Cx_ID,EV1_ADDR) != Flag_Enable);

			//EV2: 	SR1_bit6 RxNE=1 data register Not empty, cleared by writing DR register
			while(I2C_GetFlagState(I2Cx_ID,EV2_RXNE) != Flag_Enable);
		}
		data = I2Cx->DR;
	}
	return data;
}



void I2C_Generate_Start(uint8 I2Cx_ID,Flag_State_e state,Repeated_Start_e Start){
	I2C_TypeDef *I2Cx;
	I2Cx = GET_I2C_BASE_ADD(I2Cx_ID);
	if(Start == START_GEN){
		//Wait if bus is busy
		while(I2C_GetFlagState(I2Cx_ID,BUSY_FlAG));
	}
	//Enable/Disable start condition
	I2Cx->CR1.bit.START = state;
}




void I2C_Generate_Stop(uint8 I2Cx_ID,Flag_State_e state){
	I2C_TypeDef *I2Cx;
	I2Cx = GET_I2C_BASE_ADD(I2Cx_ID);
	//Enable/Disable start condition
	I2Cx->CR1.bit.STOP = state;
}




void I2C_SendSlaveAddress(uint8 I2Cx_ID,uint16 SlaveAdd,I2C_Directio_e direction){
	I2C_TypeDef *I2Cx = GET_I2C_BASE_ADD(I2Cx_ID);
	if(Global_I2C_Config[I2Cx_ID].Slave_Address_Config.I2C_Slave_Address_Mode == I2C_Slave_Address_7_Bit){
		I2Cx->DR = (((SlaveAdd & 0x7f)<< 1) | (direction  & 0x01) ) ;
	}else{
		//10 Bit Address Not Supported Yet
	}
}




Flag_State_e I2C_GetFlagState(uint8 I2Cx_ID,I2C_Flags_e Flag){
	I2C_TypeDef *I2Cx =GET_I2C_BASE_ADD(I2Cx_ID);
	Flag_State_e FlagState=0;
	uint32 Flag32=0;
	volatile uint32 DummyRead=0;

	switch(Flag){
	case BUSY_FlAG:
		FlagState = I2Cx->SR2.bit.BUSY;
		break;
	case MSL_TRA_BUSY_TXE:
		Flag32 = (((I2Cx->SR2.ALL_REG <<16) | (I2Cx->SR1.ALL_REG) ) & MSL_TRA_BUSY_TXE);
		if(Flag32 == MSL_TRA_BUSY_TXE){
			FlagState = Flag_Enable;
		}else{
			FlagState = Flag_Disbale;
		}
		break;

	default:
		FlagState = GET_BIT(I2Cx->SR1.ALL_REG,Flag);
		if(Flag == EV6_ADDR || Flag == EV1_ADDR){
			DummyRead = I2Cx->SR2.ALL_REG;
		}
		break;

	}
	return FlagState;
}





void MCAL_I2C_InterruptEnable(uint8 I2Cx_ID,I2C_IRQ_e IRQ)
{
	I2C_TypeDef *I2Cx;
	boolean error=FALSE;
	I2Cx = GET_I2C_BASE_ADD(I2Cx_ID);
	if(I2Cx == NULL){
		error = TRUE;
	}
	if(error == FALSE){
		switch(IRQ){
		case I2C_IRQ_ALL_EVENT:
		case I2C_IRQ_EVENT_WITHOUT_TX_RX:
			if(I2Cx_ID == I2C1_ID){
				NVIC_IRQ31_I2C1_EV_Enable();
			}else if(I2Cx_ID == I2C2_ID){
				NVIC_IRQ33_I2C2_EV_Enable();
			}
			I2Cx->CR2.bit.ITEVTEN = TRUE;
			if(IRQ == I2C_IRQ_ALL_EVENT){
				I2Cx->CR2.bit.ITBUFEN = TRUE;
			}
		case I2C_IRQ_ALL_ERROR:
			if(I2Cx_ID == I2C1_ID){
				NVIC_IRQ32_I2C1_ER_Enable();
			}else if(I2Cx_ID == I2C2_ID){
				NVIC_IRQ34_I2C2_ER_Enable();
			}
			I2Cx->CR2.bit.ITERREN = TRUE;
			break;
		}
	}
}

void MCAL_I2C_InterruptDisable(uint8 I2Cx_ID){
	I2C_TypeDef *I2Cx;
	boolean error=FALSE;
	I2Cx = GET_I2C_BASE_ADD(I2Cx_ID);
	if(I2Cx == NULL){
		error = TRUE;
	}
	if(error == FALSE){
		I2Cx->CR2.bit.ITBUFEN = FALSE;
		I2Cx->CR2.bit.ITERREN = FALSE;
		I2Cx->CR2.bit.ITEVTEN = FALSE;
		if(I2Cx_ID == I2C1_ID){
			NVIC_IRQ31_I2C1_EV_Disable();
			NVIC_IRQ32_I2C1_ER_Disable();
		}else if(I2Cx_ID == I2C2_ID){
			NVIC_IRQ33_I2C2_EV_Disable();
			NVIC_IRQ34_I2C2_ER_Disable();
		}
	}
}


static void SlaveStates(uint8 I2Cx_ID,I2C_IRQ_Source_e source){
//To do
}

static void MasterStates(uint8 I2Cx_ID,I2C_IRQ_Source_e source){
//To do
}

void MCAL_I2C_SetCallBackFun(uint8 I2Cx_ID,void(*P_call_back)(I2C_IRQ_Source_e Soruce))
{
	if(P_call_back != NULL && I2Cx_ID <= I2C2_ID ){
		GP_CallBackFun[I2Cx_ID]=P_call_back;
	}
}



void I2C1_EV_IRQHandler(){

}

void I2C1_ER_IRQHandler(){

}
