/*
 * nrf24l01.h
 *
 *  Created on: 7 Nov 2023
 *      Author: memoz
 */

#ifndef INCLUDES_NRF24L01_H_
#define INCLUDES_NRF24L01_H_


#include "stm32f103x6.h"
#include "GPIO.h"
#include "SPI.h"
#include "EXTI.h"
#include "stdint.h"
#include <stdbool.h>


#define NRF_clck 0
//===================================
//Configuration Reference Macros
//===================================
#define Start_up_Delay				2 //in milli Sec.


//registers definition section
#define CONFIG_ADDRESS              0X00
#define EN_AA_ADDRESS               0X01              //enable auto acknowledgment feature
#define EN_RXADDR_ADDRESS           0X02              //register containing bits to enable 6 data pipes individually
#define SETUP_AW_ADDRESS            0X03              //address field length is configured in here to be 3, 4 or 5 bytes long
#define SETUP_RETR_ADDRESS          0X04              //setup ARC bits to configure auto retransmission count
#define RF_CH_ADDRESS               0X05			  //Sets the frequency channel nRF24L01+ operates on
#define RF_SETUP_ADDRESS            0X06
#define STATUS_ADDRESS              0X07              //contains RX_DR, TX_DS, MAX_RT, RX_P_NO, TX_FULL, send R_REGISTER then NOP to read
#define OBSERVE_TX_ADDRESS          0X08              //contains ARC_CNT and PLOS_CNT, two counters for retransmission. these counters could be used to assess the network quality
#define RPD_REG_ADDRESS             0X09
#define RX_ADDR_P0_ADDRESS          0X0A              //the address for PRX device. if a packet contains this address, enhanced shockburst starts validating the packet
#define RX_ADDR_P1_ADDRESS          0X0B              //a total of 6 unique addresses could be assigned to a PRX device (Multiceiver feature)
#define RX_ADDR_P2_ADDRESS          0X0C              //these addresses must NOT be the same
#define RX_ADDR_P3_ADDRESS          0X0D
#define RX_ADDR_P4_ADDRESS          0X0E
#define RX_ADDR_P5_ADDRESS          0X0F
#define TX_ADDR_ADDRESS             0X10              //40 bits long register, transmit address, used for a PTX device only. configure address length in SETUP_AW register. set RX_ADDR_P0 equal to this address to handle automatic acknowledge
#define RX_PW_P0_ADDRESS            0X11              //these registers are for setting the static payload length in static payload length mode (receiver side)
#define RX_PW_P1_ADDRESS            0X12
#define RX_PW_P2_ADDRESS            0X13
#define RX_PW_P3_ADDRESS            0X14
#define RX_PW_P4_ADDRESS            0X15
#define RX_PW_P5_ADDRESS            0X16
#define FIFO_STATUS_ADDRESS         0X17
#define DYNPD_ADDRESS               0X1C              //on receiver side (RX mode), this register must be set to enable dynamic payload length. a PTX in dynamic mode, must have the DYNPD_P0 set
#define FEATURE_ADDRESS             0X1D              //contains the EN_DPL bit to enable dynamic payload length

//=============================================================================================================================================================================

/*bits definition section*/
//-----------------------------------------------
//Register CONFIG
#define MASK_RX_DR          (1<<6)               /*mask interrupt caused by RX_DR: 1 interrupt not reflected on IRQ pin (IRQ is active low), inside CONFIG register*/
#define MASK_TX_DS          (1<<5)               /*mask interrupt caused by TX_DS: 1 interrupt not reflected on IRQ pin (IRQ is active low), inside CONFIG register*/
#define MASK_MAX_RT         (1<<4)               /*mask interrupt caused by MAX_RT means maximum number of retransmissions reached: 1 interrupt not reflected on IRQ pin (IRQ is active low), inside CONFIG register*/
#define EN_CRC              (1<<3)               /*Enable CRC, forced high if one of the bits in EN_AA is high, inside CONFIG register*/
#define CRCO                (1<<2)               /*CRC encoding scheme, 0 is 1 byte, 1 is 2 bytes, inside CONFIG register*/
#define PWR_UP              (1<<1)               /*1 is power up, inside CONFIG register*/
#define PRIM_RX             (1<<0)               /*RX/TX control, 1: PRX, inside CONFIG register*/
//-----------------------------------------------
//Register EN_AA
#define ENAA_P5             (1<<5)               /*enable auto acknowledgment data pipe 5*/
#define ENAA_P4             (1<<4)
#define ENAA_P3             (1<<3)
#define ENAA_P2             (1<<2)
#define ENAA_P1             (1<<1)
#define ENAA_P0             (1<<0)
//-----------------------------------------------
//Register EN_RXADDR
#define ERX_P5              (1<<5)               /*part of EN_RXADDR, enable data pipe 5*/
#define ERX_P4              (1<<4)
#define ERX_P3              (1<<3)
#define ERX_P2              (1<<2)
#define ERX_P1              (1<<1)
#define ERX_P0              (1<<0)
//-----------------------------------------------
//Register SETUP_AW
#define AW_1                (1<<2)               /*RX/TX address field width, 00 illegal, 01 3 bytes, 10 4 bytes, 11 5 bytes*/
//-----------------------------------------------
//Register SETUP_RETR
#define ARD                 (1<<4)               /*auto retransmit delay, 0000 250us, 0001 500us ...> 1111 4000us*/
#define ARC                 (1<<0) 			   /*auto retransmit count, 0000 retransmit disabled, 1111 up to 15 retransmit on failure of AA. (inside SETUP_RETR register)*/
//-----------------------------------------------
//Register RF_CH
#define RF_CH_6             (1<<0)               /*sets the frequency channel nRF24L01+ operates on
 	 	 	 	 	 	 	 	 	 	 	 	   you can only write on 6 bit, bit 7 is reserved */
//-----------------------------------------------
//Register RF_SETUP
#define CONT_WAVE           (1<<7)               /*enables continuous carrier transmit when high*/
#define RF_DR_LOW           (1<<5)               /*sets the RF data rate to 250kbps*/
#define PLL_LOCK            (1<<4)               /*force PLL lock signal. used for testing ONLY*/
#define RF_DR_HIGH          (1<<3)               /*select between high speed data rates and works ONLY when RF_DR_LOW is 0. 0 for 1Mbps, 1 for 2Mbps*/
#define RF_PWR_1            (1<<2)
#define RF_PWR_0            (1<<1)
//-----------------------------------------------
//Register STATUS
#define RX_DR               (1<<6)               /*IRQ for new packet in RX FIFO (newly received)*/
#define TX_DS               (1<<5)               /*IRQ for ACK received in TX mode*/
#define MAX_RT              (1<<4)
#define RX_P_NO_2           (1<<3)
#define RX_P_NO_1           (1<<2)
#define RX_P_NO_0           (1<<1)
#define TX_IRQ_FULL         (1<<0)
//-----------------------------------------------
//Register OBSERVE_TX
#define PLOS_CNT_3          (1<<7)               /*inside OBSERVE_TX register, counts the total number of retransmissions since last channel change. reset by writing to RF_CH*/
#define PLOS_CNT_2          (1<<6)
#define PLOS_CNT_1          (1<<5)
#define PLOS_CNT_0          (1<<4)
#define ARC_CNT_3           (1<<3)               /*inside OBSERVE_TX register, counts the number of retransmissions for current transaction. reset by initiating new transaction*/
#define ARC_CNT_2           (1<<2)
#define ARC_CNT_1           (1<<1)
#define ARC_CNT_0           (1<<0)
//-----------------------------------------------
//Register RPD
#define RPD                 (1<<0)               /*received power detector, if received power is less than -64dbm, RPD = 0*/
//-----------------------------------------------
//Register FIFO_STATUS
#define TX_REUSE            (1<<6)
#define TX_FULL             (1<<5)
#define TX_EMPTY            (1<<4)
#define RX_FULL             (1<<1)
#define RX_EMPTY            (1<<0)
//-----------------------------------------------
//Register DYNPD
#define DPL_P5              (1<<5)
#define DPL_P4              (1<<4)
#define DPL_P3              (1<<3)
#define DPL_P2              (1<<2)
#define DPL_P1              (1<<1)
#define DPL_P0              (1<<0)                 /*must be set on PTX in dynamic payload length mode*/
//-----------------------------------------------
//Register FEATURE
#define EN_DPL              (1<<2)                 /*set to enable dynamic payload length*/
#define EN_ACK_PAY          (1<<1)                 /*used to enable auto acknowledgment with payload in PRX (inside FEATURE register)*/
#define EN_DYN_ACK          (1<<0)                 /**/
//===============================================================================================================================================

/*commands definition section*/
#define R_REGISTER          	  0X00              /*read command and STATUS registers, 5 bit register map address followed by register ADD*/
#define W_REGISTER          	  0X20              /*write command and STATUS registers, 5 bit register map address followed by register ADD, executable in POWER DOWN or STANDBY modes only*/
#define R_RX_PAYLOAD        	  0X61              /*read RX payload, 1-32 bytes. read operation starts at byte 0. payload is deleted from FIFO after its read*/
#define W_TX_PAYLOAD        	  0XA0              /*write TX payload, starts at byte 0, 1-32 bytes*/
#define FLUSH_TX            	  0XE1              /*flush TX FIFO, used in TX mode*/
#define FLUSH_RX            	  0XE2              /*flush RX FIFO, used in RX mode*/
#define REUSE_TX_PL         	  0XE3              /*used for a PTX device, reuse last transmitted payload for an exact number. alternative to auto retransmission*/
#define R_RX_PL_WID         	  0X60              /*command for receiver side, in order to read the payload length in dynamic payload length mode*/
#define W_ACK_PAYLOAD_PIPE0       0XA8              /*used in RX mode, to write payload in TX FIFO and later transmit the payloads along with ACK packet to PTX, if DPL is enabled
 	 	 	 	 	 	 	 	 	 	 	    	this in binary equals to 1010 1PPP (PPP can be range from 000 to 101 depend on which pipe is EN)*/
#define W_ACK_PAYLOAD_PIPE1       0XA9
#define W_ACK_PAYLOAD_PIPE2       0XAA
#define W_ACK_PAYLOAD_PIPE3       0XAB
#define W_ACK_PAYLOAD_PIPE4       0XAC
#define W_ACK_PAYLOAD_PIPE5       0XAD
#define W_TX_PAYLOAD_NOACK  	  0XB0              /*used in TX mode, disables AUTOACK on this specific packet. must be first enabled in FEATURE register by setting the EN_DYN_ACK bit. if used, PTX will not wait for ACK and goes directly to standby I*/
#define NOP_CMD             	  0XFF              /*might be used to read the status register*/
#define ACTIVATE				  0x50
#define ACTIVATE_BYTE             0x73  // the activate command must be followed by this byte
//---------------Custom Defines -----------------------
//I like this naming convention better and can apply to all the enable registers
#define PIPE_0	(1<<0x00)
#define PIPE_1	(1<<0x01)
#define PIPE_2	(1<<0x02)
#define PIPE_3	(1<<0x03)
#define PIPE_4	(1<<0x04)
#define PIPE_5	(1<<0x05)
//some offsets that make fr more readability
#define RX_ADDR_OFFSET  0x0A
#define RX_PW_OFFSET    0x11
//SETUP_AW byte codes
#define THREE_BYTES 0b01
#define FOUR_BYTES  0b10
#define FIVE_BYTES  0b11
#define ENCODING_SCHEME_1_BYTE  0x00
#define ENCODING_SCHEME_2_BYTE  0x01

#define DUMMYBYTE  0xF1 // DUMMYBYTE  is just a define used when I need to send a dummy byte via SPI because remember when you read from a register, you first send the command to read the register then you need to clock in some data so that the device can
						//-send you the data you want to read, the data you send doesnt matter, what matters is that you are clocking the SPI


//====================================================================================================================================================

typedef struct //user structure to setup and control the NRF24
{
	//common settings
	bool		set_enable_crc;                 // true / false
	uint8_t		set_crc_scheme;			// ENCODING_SCHEME_1_BYTE / ENCODING_SCHEME_2_BYTE
	bool		set_enable_auto_ack;		// true / false
	uint8_t		set_rf_channel;			// 0 - 126  (2.400GHz to 2.525GHz)(2400 + RF_CH)
	uint8_t		set_address_width;		// THREE_BYTES / FOUR_BYTES / FIVE_BYTES
	bool		set_enable_dynamic_pl_width;	// true / false

	//rx settings
	bool		set_enable_rx_mode;		// true / false
	uint8_t		set_rx_pipe;			// must set depend on EN_RXADDR
	uint8_t		set_rx_addr_byte_1;		// low byte will go in RX_ADDR_P#
	uint32_t	set_rx_addr_byte_2_5;		// high byte will go in Pipe 1
	uint8_t		set_payload_width;		// 1 - 32
	bool		set_enable_rx_dr_interrupt;	// true / false

	//tx settings
	bool		set_enable_tx_mode;		// true / false
	uint8_t		set_tx_addr_byte_1;		// low byte will go in TX_ADDR register
	uint32_t	set_tx_addr_byte_2_5;		// high byte low byte will go in TX_ADDR register
											// and PIPE 0 if auto ack is enabled
	bool 		en_continous_transmit;				//enable continuous carrier transmission
	bool		set_enable_max_rt_interrupt;	// true / false
	bool		set_enable_tx_ds_interrupt;	// true / false


}CL_nrf24l01p_init_type;

//-----------| NRF Macros / Variables |-----------
#define NRF_SPI			SPI1_ID
#define NRF_CE_PIN		GPIO_PIN_2  //chip enable pin to make the transceiver transmit or listen
#define NRF_CE_PORT		GPIO_PORTA
#define NRF_IRQ_PIN		GPIO_PIN_4 // [ interrupt pin active Low ]
#define NRF_IRQ_PORT	GPIO_PORTA
#define NRF_CSN_PIN		GPIO_PIN_3 //Low when SPI active
#define NRF_CSN_PORT    GPIO_PORTA

#define NRF_CLK_PIN		GPIO_PIN_5
#define NRF_MOSI_PIN	GPIO_PIN_7
#define NRF_MISO_PIN	GPIO_PIN_6
#define NRF_PINS_CLOCK_ENABLE() RCC_GPIOA_CLK_EN() //given the pins are on GPIOA
#define NVIC_nRF_IRQ_EN()		NVIC_IRQ7_EXTI1_Enable()

uint8_t tx_data_buff[32];
uint8_t rx_data_buff[32];



typedef enum
{
POWER_DOWN,
STANDBY_I,
STANDBY_II,
PTX,
PRX,
DEVICE_NOT_INITIALIZED
}nRF_State;

typedef enum
{
Enable,
Disable
}nRF_dyn_ACK;

typedef enum
{
    _250kbps = 2,
    _1Mbps   = 0,
    _2Mbps   = 1
} air_data_rate;

typedef enum
{
    _0dBm  = 3,
    _6dBm  = 2,
    _12dBm = 1,
    _18dBm = 0
} output_power;

/*
 *========================================================================================
 * 							APIs Supported by "HAL nRF24lo1 DRIVER"
 *========================================================================================
 */

void nrf24_read(uint8_t address, uint8_t *value, uint8_t data_length);
void nrf24_write(uint8_t address, uint8_t *value, uint8_t data_length);
void init_nRFpins(void);
void CE_pin_HIGH(void);
void CE_pin_LOW(void);
void CSN_pin_HIGH(void);
void CSN_pin_LOW(void);
void init_nrfspi(void);




void NRF_cmd_get_pipe_current_pl(uint8_t *value);
void  NRF_cmd_read_dynamic_pl_width(uint8_t *value);
void NRF_cmd_get_status(uint8_t *value);
void NRF_set_tx_addr(uint32_t addr_high, uint8_t addr_low, bool auto_ack);
void NRF_set_rx_addr(uint8_t rx_pipe, uint32_t addr_high, uint8_t addr_low);
void NRF_cmd_read_RX_PAYLOAD(uint8_t *data, uint8_t len);
void NRF_cmd_write_TX_PAYLOAD(uint8_t *data, uint8_t len);
void NRF_cmd_setup_addr_width(uint8_t *width);
void NRF_cmd_FLUSH_TX(void);
void NRF_cmd_FLUSH_RX(void);
void NRF_cmd_reuse_TX_PL(void);
void NRF_cmd_activate(void);
void NRF_cmd_listen(void);
void NRF_cmd_clear_interrupts(void);
void NRF_cmd_act_as_RX(bool state);
void NRF_init(CL_nrf24l01p_init_type *nrf_type);
void NRF_set_tx_addr(uint32_t addr_high, uint8_t addr_low , bool auto_ack);
void NRF_set_rx_addr(uint8_t rx_pipe, uint32_t addr_high, uint8_t addr_low);
void nRF24lo1_TXRX_init(nRF_State state);
void NRF_cmd_listen(void);
void nrf_exti_callback(void);
#endif /* INCLUDES_NRF24L01_H_ */
