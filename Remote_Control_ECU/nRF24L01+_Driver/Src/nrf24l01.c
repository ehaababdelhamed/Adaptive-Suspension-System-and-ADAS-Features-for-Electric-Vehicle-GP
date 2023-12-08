/*
 * nrf24l01.c
 *
 *  Created on: 7 Nov 2023
 *      Author: memoz
 */

#include "nrf24l01.h"

static uint16_t SPI_command;

void delay_ms(unsigned int ms) {
	unsigned int i, j;
	for (i = 0; i < ms; i++) {
		// Adjust the inner loop to provide an approximate delay of 1 millisecond
		for (j = 0; j < NRF_clck; j++) {
			// This value of 16000 is an approximation for 1ms delay at a 16MHz clock frequency
			// Actual delay might vary and require fine-tuning based on the MCU and compiler used
		}
	}
}

/************************************************************************************
 * Function Name	: nrf24_read
 * Description		: send a read command followed by address u want to read from
 * Parameters (in)	: address- Address u want to read
 * 					  *value	-buffer u want to save in it data
 * 					  data_length - the length of data u want to read
 * Return value		: none
 * Note				: None
 ************************************************************************************/
void nrf24_read(uint8_t address, uint8_t *value, uint8_t data_length)
{
	CSN_pin_LOW();
	SPI_command = R_REGISTER | address;    /*in order to read CONFIG, then change one bit*/
	MCAL_SPI_sendData(NRF_SPI,SPI_command,SPI_POLLING_DISABLE);
	SPI_command = NOP_CMD;
	for (; data_length ; data_length--)
	{
		*value=MCAL_SPI_receiveData (NRF_SPI,SPI_POLLING_DISABLE );
		value++;
	}
	CSN_pin_HIGH();

}

/************************************************************************************
 * Function Name	: nrf24_write
 * Description		: send a write command followed by address u want to read from
 * Parameters (in)	: address- Address u want to write
 * 					  *value	-buffer contains the data u want to write
 * 					  data_length - the length of data u want to write
 * Return value		: none
 * Note				: None
 ************************************************************************************/
void nrf24_write(uint8_t address, uint8_t *value, uint8_t data_length)
{
	CSN_pin_LOW();


	SPI_command = W_REGISTER | address;    /*in order to read CONFIG, then change one bit*/
	MCAL_SPI_sendData(NRF_SPI,SPI_command,SPI_POLLING_DISABLE);
	for (; data_length ; data_length--)
	{
		MCAL_SPI_sendData(NRF_SPI,*value,SPI_POLLING_DISABLE);
		value++;

	}
	CSN_pin_HIGH();

}
/************************************************************************************
 * Function Name	: nrf24_write
 * Description		: send a write command followed by address u want to read from
 * Parameters (in)	: address- Address u want to write
 * 					  *value	-buffer contains the data u want to write
 * 					  data_length - the length of data u want to write
 * Return value		: none
 * Note				: None
 ************************************************************************************/
void init_nRFpins(void)
{
	//clock enable GPIOA
	NRF_PINS_CLOCK_ENABLE();
	RCC_AFIO_CLK_EN();

	GPIO_PinConfig_t nrfPins;
	EXTI_Config_t nrf_EXIT;


	// CE pin as output : active high/ normally low
	nrfPins.Pin_Num			= NRF_CE_PIN;
	nrfPins.Pin_Mode		= GPIO_OUTPUT_PP;
	nrfPins.Pin_Speed		= GPIO_SPEED_10_MHZ;
	MCAL_GPIO_init(NRF_CE_PORT, &nrfPins); // CE on same port only need this once


	// CSN chip select for spi active low / normally high
	nrfPins.Pin_Num			= NRF_CSN_PIN;
	nrfPins.Pin_Mode		= GPIO_OUTPUT_PP;
	nrfPins.Pin_Speed		= GPIO_SPEED_10_MHZ;
	MCAL_GPIO_init(NRF_CSN_PORT, &nrfPins); // CSN on same port only need this once

	// IRQ pin as input with interrupt enabled
	nrf_EXIT.EXTI_Pin.GPIO_Pin = NRF_IRQ_PIN;
	nrf_EXIT.EXTI_Pin.GPIO_Port = NRF_IRQ_PORT;
	nrf_EXIT.EXTI_Pin.IRQ_Num = EXTI1;
	nrf_EXIT.EXTI_Pin.IRQ_Num= EXTI1_IRQ;
	nrf_EXIT.Edge = FAILING_EGDE;

	MCAL_EXTI_init(&nrf_EXIT);
	MCAL_EXTI_setCallBackFun(nrf_EXIT.EXTI_Pin.IRQ_Num,nrf_exti_callback);

}
/************************************************************************************
 * Function Name	: CE_pin_HIGH ,CE_pin_LOW,CSN_pin_HIGH,CSN_pin_LOW
 * Description		: set or reset csn and ce
 * Parameters (in)	: none
 * Return value		: none
 * Note				: None
 ************************************************************************************/
void CE_pin_HIGH(void)
{
	//TO be discussed
	GPIOA->BSRR |= NRF_CE_PIN;
}
void CE_pin_LOW(void)
{
	GPIOA->BRR |= NRF_CE_PIN;
}
void CSN_pin_HIGH(void)
{
	GPIOA->BSRR |= NRF_CSN_PIN;
}
void CSN_pin_LOW(void)
{
	GPIOA->BRR |= NRF_CSN_PIN;
}
/************************************************************************************
 * Function Name	: init_nrfspi
 * Description		: init spi used to communicate with nrf
 * Parameters (in)	: none
 * Return value		: none
 * Note				: None
 ************************************************************************************/
void init_nrfspi(void)
{
	// CLOCK  [ Alt Function ] [ GPIOA ] [ SPI1 ]
	RCC_AFIO_CLK_EN();
	RCC_GPIOA_CLK_EN();
	RCC_SPI1_CLK_EN();


	// SPI
	SPI_Config_t nrf_SPI;
	nrf_SPI.SPI_Mode = SPI_MODE_FULL_DUPLEX;
	nrf_SPI.Device_Mode = SPI_MASTER ;
	nrf_SPI.SPI_Prescaler = SPI_PERSCALER_2;
	nrf_SPI.Data_Size = SPI_DATA_SIZE_8B;
	nrf_SPI.Data_Order = SPI_DATA_ORDER_MSB_FIRST;
	nrf_SPI.Clock_Polarity = 	SPI_CLKP_IDLE_HIGH;
	nrf_SPI.Clock_Phase = SPI_CLKPh_SECOND_EDGE;
	nrf_SPI.NSS_Mode= SPI_NSS_SW_SET;

	MCAL_SPI_init(NRF_SPI,&nrf_SPI);
	MCAL_SPI_GPIO_SET_PINS(NRF_SPI);

}
/************************************************************************************
 * Function Name	: NRF_cmd_modify_reg
 * Description		: this function used to modify a specific Reg
 * Parameters (in)	: reg -Register u want to modify
 * 					  bit	-bit u want to set or reset
 * 					  state - state 1 or 0
 * Return value		: none
 * Note				: None
 ************************************************************************************/
void NRF_cmd_modify_reg(uint8_t reg, uint8_t bit, uint8_t state)
{
	// this is a "read-modify-write" procedure


	//READ
	uint8_t *reg_value = NULL ;
	nrf24_read(reg,reg_value,1);

	//MODIFY
	if (state)
	{
		*reg_value |= (bit);
	}
	else
	{
		*reg_value &= ~(bit);
	}

	//WRITE
	nrf24_write(reg,reg_value,1);
}
/************************************************************************************
 * Function Name	: NRF_cmd_get_status
 * Description		:this function used to get nrf status
 * Parameters (in)	: *value- to read the status
 * Return value		: none
 * Note				: None
 ************************************************************************************/
void NRF_cmd_get_status(uint8_t *value)
{
	nrf24_read(STATUS_ADDRESS, value, 1);

}
/************************************************************************************
 * Function Name	: NRF_cmd_clear_interrupts
 * Description		: this function used to clear all interrupts
 * Parameters (in)	: address- Address u want to write
 * 					  *value	-buffer contains the data u want to write
 * 					  data_length - the length of data u want to write
 * Return value		: none
 * Note				: None
 ************************************************************************************/
void NRF_cmd_clear_interrupts(void)
{
	uint8_t value=0x70;
	nrf24_write(STATUS_ADDRESS, &value,1);
}
/************************************************************************************
 * Function Name	: nrf24_write
 * Description		: send a write command followed by address u want to read from
 * Parameters (in)	: address- Address u want to write
 * 					  *value	-buffer contains the data u want to write
 * 					  data_length - the length of data u want to write
 * Return value		: none
 * Note				: None
 ************************************************************************************/
void NRF_cmd_get_pipe_current_pl(uint8_t *value)
{
	nrf24_read(STATUS_ADDRESS, value, 1);
	*value &= 0b00001110;
}
/************************************************************************************
 * Function Name	: NRF_cmd_FLUSH_TX
 * Description		: flush TX fifo
 * Parameters (in)	: none
 * Return value		: none
 * Note				: None
 ************************************************************************************/
void NRF_cmd_FLUSH_TX(void)
{
	CSN_pin_LOW();

	SPI_command = FLUSH_TX;
	MCAL_SPI_sendData(NRF_SPI,SPI_command,SPI_POLLING_DISABLE);

	CSN_pin_HIGH();
}
/************************************************************************************
 * Function Name	: NRF_cmd_FLUSH_RX
 * Description		: flush RX fifo
 * Parameters (in)	: none
 * Return value		: none
 * Note				: None
 ************************************************************************************/
void NRF_cmd_FLUSH_RX(void)
{
	CSN_pin_LOW();


	SPI_command = FLUSH_RX;
	MCAL_SPI_sendData(NRF_SPI,SPI_command,SPI_POLLING_DISABLE);

	CSN_pin_HIGH();
}
/************************************************************************************
 * Function Name	: NRF_cmd_act_as_RX
 * Description		: make nrf act as transmitter or receiver
 * Parameters (in)	: address- Address u want to write
 * 					  *value	-buffer contains the data u want to write
 * 					  data_length - the length of data u want to write
 * Return value		: none
 * Note				: None
 ************************************************************************************/
void NRF_cmd_act_as_RX(bool state)
{
	if (state)
	{
		NRF_cmd_modify_reg(CONFIG_ADDRESS, PRIM_RX, 1);
	}

	else
	{
		NRF_cmd_modify_reg(CONFIG_ADDRESS, PRIM_RX, 0);
	}

}
/************************************************************************************
 * Function Name	: NRF_cmd_read_dynamic_pl_width
 * Description		: read the dynamic payload width
 * Parameters (in)	: *value- buffer to store the width in
 * Return value		: none
 * Note				: None
 ************************************************************************************/
void NRF_cmd_read_dynamic_pl_width(uint8_t *value)
{
	SPI_command = R_RX_PL_WID ;
	MCAL_SPI_sendData (NRF_SPI,SPI_command,SPI_POLLING_DISABLE);
	*value=MCAL_SPI_receiveData(NRF_SPI,SPI_POLLING_DISABLE );

}
/************************************************************************************
 * Function Name	: NRF_cmd_setup_addr_width
 * Description		: Setting address width
 * Parameters (in)	: *width- width of Address
 * Return value		: none
 * Note				: None
 ************************************************************************************/
void NRF_cmd_setup_addr_width(uint8_t *width)
{
	uint8 _3= 0b00000001;
	uint8 _4= 0b00000010;
	uint8 _5= 0b00000011;
	switch(*width)
	{
	case 4:
		nrf24_write(SETUP_AW_ADDRESS, &_4,1);
		break;
	case 5:
		nrf24_write(SETUP_AW_ADDRESS, &_5,1);
		break;
	default:
		nrf24_write(SETUP_AW_ADDRESS, &_3,1);
		break;
	}
}
/************************************************************************************
 * Function Name	: NRF_cmd_activate
 * Description		: Activating Commands
 * Parameters (in)	: none
 * Return value		: none
 * Note				: None
 ************************************************************************************/
void NRF_cmd_activate(void)
{
	CSN_pin_LOW();


	SPI_command = ACTIVATE;
	MCAL_SPI_sendData(NRF_SPI,SPI_command,SPI_POLLING_DISABLE);

	SPI_command = ACTIVATE_BYTE;
	MCAL_SPI_sendData(NRF_SPI,SPI_command,SPI_POLLING_DISABLE);

	CSN_pin_HIGH();
}
/************************************************************************************
 * Function Name	: NRF_set_tx_addr
 * Description		: setting transmitter address
 * Parameters (in)	: addr_high- last 4 bytes
 * 					  addr_low	-first byte
 * 					  auto_ack - enable or disable auto ack
 * Return value		: none
 * Note				: None
 ************************************************************************************/
void NRF_set_tx_addr(uint32_t addr_high, uint8_t addr_low, bool auto_ack)
{
	uint32_t temp;
	CSN_pin_LOW(); //start SPI comms by a LOW on CSN
	/*  5 byte address is divided into a uint8_t low byte
	 *  and a uint32_t high byte
	 *  since the SPI can only send 1 byte at a time
	 *  we first send the low byte
	 *  and then extract the other bytes from the uint32
	 *  and send them one by one LSB first
	 */
	nrf24_write(TX_ADDR_ADDRESS,&addr_low,1 ); //send write command to ADDR and addr_low
	temp = (addr_high & 0xFF);
	MCAL_SPI_sendData (NRF_SPI,temp,SPI_POLLING_DISABLE);
	temp =((addr_high >> 8) & 0xFF);
	MCAL_SPI_sendData (NRF_SPI,temp,SPI_POLLING_DISABLE);
	temp =((addr_high >> 16) & 0xFF);
	MCAL_SPI_sendData (NRF_SPI,temp,SPI_POLLING_DISABLE);
	temp =((addr_high >> 24) & 0xFF);
	MCAL_SPI_sendData (NRF_SPI,temp,SPI_POLLING_DISABLE);
	CSN_pin_HIGH();

	/* If auto ack is enabled then the same address that was written
	 * to TX_ADDR above must also be written to PIPE 0 because that
	 * is the pipe that it will receive the auto ack on. This cannot
	 * be changed it is hardwaired to receive acks on pipe 0 */

	if (auto_ack)
	{
		NRF_cmd_modify_reg(EN_AA_ADDRESS, ENAA_P0, 1); //enable auto ack on pipe 0
		CSN_pin_LOW();

		//write address into pipe 0
		nrf24_write(RX_PW_P0_ADDRESS,&addr_low,1 );

		temp = (addr_high & 0xFF);
		MCAL_SPI_sendData (NRF_SPI,temp,SPI_POLLING_DISABLE);
		temp =((addr_high >> 8) & 0xFF);
		MCAL_SPI_sendData (NRF_SPI,temp,SPI_POLLING_DISABLE);
		temp =((addr_high >> 16) & 0xFF);
		MCAL_SPI_sendData (NRF_SPI,temp,SPI_POLLING_DISABLE);
		temp =((addr_high >> 24) & 0xFF);
		MCAL_SPI_sendData (NRF_SPI,temp,SPI_POLLING_DISABLE);


		CSN_pin_HIGH(); //end spi
	}
}

/************************************************************************************
 * Function Name	: NRF_cmd_read_RX_PAYLOAD
 * Description		: Read the received load
 * Parameters (in)	: *rx_buffer- buffer to save data in
 * 					 len	-length of the received data
 * Return value		: none
 * Note				: None
 ************************************************************************************/
void NRF_cmd_read_RX_PAYLOAD(uint8_t *rx_buffer, uint8_t len)
{
	CE_pin_LOW();

	CSN_pin_LOW();
	uint16_t command = R_RX_PAYLOAD;
	uint16_t dummy = DUMMYBYTE;
	uint8_t no_cmd = NOP_CMD;

	MCAL_SPI_sendData (NRF_SPI,command,SPI_POLLING_DISABLE);

	MCAL_SPI_sendData(NRF_SPI,dummy,SPI_POLLING_DISABLE);

	nrf24_read(no_cmd, rx_buffer,len);

	CSN_pin_HIGH();
}
/************************************************************************************
 * Function Name	: NRF_cmd_write_TX_PAYLOAD
 * Description		: used to send payload
 * Parameters (in)	: *data- data u want to send
 * 					  len	-Length of the data
 * Return value		: none
 * Note				: None
 ************************************************************************************/
void NRF_cmd_write_TX_PAYLOAD(uint8_t *data, uint8_t len)
{

	CE_pin_LOW();
	CSN_pin_LOW();
	uint16_t command = W_TX_PAYLOAD;
	MCAL_SPI_sendData(NRF_SPI,command,SPI_POLLING_DISABLE);

	for (; len ; len--)
	{
		MCAL_SPI_sendData(NRF_SPI,*data,SPI_POLLING_DISABLE);
		data++;
	}
	CSN_pin_HIGH();;
	//CE set high to start transmition if in TX mode
	// must be held high for a bit then back low
	CE_pin_HIGH();
	//delayUS(50); //this was here for debugging purposes feel free to delete or insert your own
	//CE_pin_LOW(); ->this used to conserve energy to set nrf in low power mode
}
/************************************************************************************
 * Function Name	: NRF_set_rx_addr
 * Description		: Setting Address for RX
 * Parameters (in)	: rx_pipe- the pipe u want to modify
 * 					  addr_high	-last 4 byte
 * 					  addr_low -first byte
 * Return value		: none
 * Note				: None
 ************************************************************************************/
void NRF_set_rx_addr(uint8_t rx_pipe, uint32_t addr_high, uint8_t addr_low)
{
	uint32_t temp;
	CSN_pin_LOW(); //start SPI comms by a LOW on CSN
	/*  5 byte address is divided into a uint8_t low byte
	 *  and a uint32_t high byte
	 *  since the SPI can only send 1 byte at a time
	 *  we first send the low byte
	 *  and then extract the other bytes from the uint32
	 *  and send them one by one LSB first
	 */
	nrf24_write(RX_ADDR_P1_ADDRESS,&addr_low,1 ); //send write command to ADDR and addr_low
	temp = (addr_high & 0xFF);
	MCAL_SPI_sendData (NRF_SPI,temp,SPI_POLLING_DISABLE);
	temp =((addr_high >> 8) & 0xFF);
	MCAL_SPI_sendData (NRF_SPI,temp,SPI_POLLING_DISABLE);
	temp =((addr_high >> 16) & 0xFF);
	MCAL_SPI_sendData (NRF_SPI,temp,SPI_POLLING_DISABLE);
	temp =((addr_high >> 24) & 0xFF);
	MCAL_SPI_sendData (NRF_SPI,temp,SPI_POLLING_DISABLE);
	CSN_pin_HIGH();
}
/************************************************************************************
 * Function Name	: NRF_init
 * Description		: send a write command followed by address u want to read from
 * Parameters (in)	: *nrf_type -Configuration structure for nrf
 * Return value		: none
 * Note				: None
 ************************************************************************************/
void NRF_init(CL_nrf24l01p_init_type *nrf_type)
{
	uint8_t reset = 0x00;
	uint8_t clr_int = 0x70;


	CSN_pin_LOW();
	//start SPI comms
	NRF_cmd_modify_reg(RF_SETUP_ADDRESS,0b00000110,1);
	NRF_cmd_modify_reg(RF_SETUP_ADDRESS,RF_DR_LOW,1);

	//common configurations
	NRF_cmd_modify_reg(CONFIG_ADDRESS, PWR_UP, 1);    // turn on
	//delayMS(100); //this was here for debugging purposes
	NRF_cmd_modify_reg(CONFIG_ADDRESS, CRCO, nrf_type->set_crc_scheme);      //set CRC scheme
	NRF_cmd_modify_reg(CONFIG_ADDRESS, EN_CRC, nrf_type->set_enable_crc);    //turn on CRC
	NRF_cmd_modify_reg(CONFIG_ADDRESS, MASK_TX_DS, !(nrf_type->set_enable_tx_ds_interrupt));    //Disable TX_DS interrupt on IRQ pin
	NRF_cmd_modify_reg(CONFIG_ADDRESS, MASK_MAX_RT, !(nrf_type->set_enable_max_rt_interrupt));   //disable MAX_RT interrupt on IRQ pin
	nrf24_write(RF_CH_ADDRESS, &(nrf_type->set_rf_channel),1);  	//rf channel
	nrf24_write(EN_AA_ADDRESS, &reset,1);     //disable auto ack by default, might be enabled below if user wants
	nrf24_write(STATUS_ADDRESS, &clr_int,1);      //clear any interrupts
	nrf24_write(SETUP_AW_ADDRESS, &(nrf_type->set_address_width),1);    //address width


	// SET UP AS RECEIVER
	if (nrf_type->set_enable_rx_mode)
	{
		NRF_cmd_modify_reg(EN_RXADDR_ADDRESS, nrf_type->set_rx_pipe, 1);    //enable rx pipe
		NRF_set_rx_addr(nrf_type->set_rx_pipe, nrf_type->set_rx_addr_byte_2_5, nrf_type->set_rx_addr_byte_1);
		NRF_cmd_modify_reg(CONFIG_ADDRESS, MASK_RX_DR, !(nrf_type->set_enable_rx_dr_interrupt));   //enable RX_DR interrupt on IRQ pin

		if (nrf_type->set_enable_dynamic_pl_width)
		{
			NRF_cmd_activate();
			NRF_cmd_modify_reg(FEATURE_ADDRESS, EN_DPL, 1); //enable dynamic PL feature
			NRF_cmd_modify_reg(DYNPD_ADDRESS, nrf_type->set_rx_pipe, 1); //enable dynamic PL for pipe
			NRF_cmd_modify_reg(DYNPD_ADDRESS, DPL_P1, 1);  //enable dynamic PL for pipe
			//auto ack MUST to be enabled if using dynamic payload
			NRF_cmd_modify_reg(EN_AA_ADDRESS, ENAA_P5, 1);//enable auto ack on pipe 1
			NRF_cmd_modify_reg(EN_AA_ADDRESS, nrf_type->set_rx_pipe, 1);
			NRF_cmd_modify_reg(EN_RXADDR_ADDRESS, nrf_type->set_rx_pipe, 1);     //enable auto ack on pipe 5
		}
		else
		{
			nrf24_write((nrf_type->set_rx_pipe + RX_PW_P0_ADDRESS), &(nrf_type->set_payload_width),1);   //write the static payload width

			//if using static PL width use can still use auto ack but now its optional
			if(nrf_type->set_enable_auto_ack)
			{
				NRF_cmd_modify_reg(EN_AA_ADDRESS, ENAA_P1, 1);       //enable auto ack on pipe 1
				NRF_cmd_modify_reg(EN_AA_ADDRESS, nrf_type->set_rx_pipe, 1);     //enable auto ack on pipe 5
			}
		}
	}
	// SET UP AS TRANSMITTER
	if(nrf_type->set_enable_tx_mode)
	{
		uint8_t setup = 0x2F;
		nrf24_write(SETUP_RETR_ADDRESS, &setup,1);

		NRF_set_tx_addr(nrf_type->set_tx_addr_byte_2_5, nrf_type->set_tx_addr_byte_1, nrf_type->set_enable_auto_ack);
		if(nrf_type->en_continous_transmit)
		{
			NRF_cmd_modify_reg(RF_SETUP_ADDRESS, CONT_WAVE, 1);
		}else
		{
			NRF_cmd_modify_reg(RF_SETUP_ADDRESS, CONT_WAVE, 0);
		}

		if (nrf_type->set_enable_dynamic_pl_width)
		{
			NRF_cmd_activate();
			NRF_cmd_modify_reg(FEATURE_ADDRESS, EN_DPL, 1);    //enable dynamic PL feature
			NRF_cmd_modify_reg(DYNPD_ADDRESS, DPL_P0, 1);
		}
	}
	CSN_pin_HIGH();
}
/************************************************************************************
 * Function Name	: nrf24_write
 * Description		: send a write command followed by address u want to read from
 * Parameters (in)	: address- Address u want to write
 * 					  *value	-buffer contains the data u want to write
 * 					  data_length - the length of data u want to write
 * Return value		: none
 * Note				: None
 ************************************************************************************/
void NRF_cmd_listen(void)
{
	CE_pin_HIGH();
}
//cmd_transmit : This command is responsible for transmitting your payload to the address specified in the TX_ADDR , its parameters include a pointer to your data

void nrf_exti_callback(void)
{


	NRF_cmd_modify_reg(STATUS_ADDRESS,RX_DR,1);
}
