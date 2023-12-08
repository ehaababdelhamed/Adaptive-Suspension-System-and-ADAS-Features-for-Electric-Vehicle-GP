/******************************************************************************
 * File Name: ADC.h
 * Description: MCU device header for ADC
 * Author: Khalid walid alazouni
 ******************************************************************************/
#ifndef ADC_H_
#define ADC_H_

/*******************************************************************************
 *                             Includes			                            	*
 *******************************************************************************/
#include "Common_Macros.h"
#include "Platform_Types.h"


/*******************************************************************************
 *                             	ADC Data Types                             	*
 *******************************************************************************/


/*Specifics which channel to read
 * ADC_channel_0-> ADC_channel_7     PA0->PA7
 * ADC_channel_8                     PB0
 * ADC_channel_9                     PB1
 */

typedef enum{
	ADC_channel_0,ADC_channel_1,ADC_channel_2,
	ADC_channel_3,ADC_channel_4,ADC_channel_5,
	ADC_channel_6,ADC_channel_7,ADC_channel_8,
	ADC_channel_9
}ADC_ChannelNum_t;


//@REF : ADC_Data_Align_t
typedef enum {
	ADC_RIGHT_ALIGN,ADC_LEFT_ALIGN
}ADC_Data_Align_t;

//@REF : ADC_SampleTime_t
typedef enum{
	ADC_1_cycles,ADC_7_cycles,ADC_13_cycles,
	ADC_28_cycles,ADC_41_cycles,ADC_55_cycles,
	ADC_71_cycles,ADC_239_cycles

}ADC_SampleTime_t;



typedef enum {
      ADC_Enable_Single_Mode,ADC_Enable_Continuous_Mode

}ADC_Mode_config_t;


//Specifics the channel configration
typedef struct {
	ADC_SampleTime_t ADC_SamleRate;     ////specifies channel sample_rate  @RF : ADC_SampleTime_t

}ADC_set_channel_config;



typedef struct {
	ADC_Mode_config_t ADC_ConvMode;           //specifies ADC mode @RF : ADC_Mode_config_t

	ADC_Data_Align_t ADC_DataAlign;          //Specifics left or right alignment @REF : ADC_Data_Align_t

	uint8 ADC_NUM_OF_CAHNNELS;              //Specifies the number of ADC channels that will be converted  using the sequencer
	                                        //varies from 1->10


}ADC_Config_t;


/*******************************************************************************
 *                             	ADC Definition                          		*
 *******************************************************************************/

//Specifics which ADC_instance to use
typedef enum
{
	ADC1_ID,
	ADC2_ID

}ADC_instance_t;

/*******************************************************************************
 *					APIs Supported by "MCAL I2C DRIVER" 	                   *
 *******************************************************************************/

void MCAL_ADC_init(ADC_instance_t ADCx_ID,ADC_Config_t *ADCx_Config);
void MCAL_ADC_Dinit(ADC_instance_t ADCx_ID);
uint32 MCAL_ADC_Read(ADC_instance_t ADCx_ID , ADC_ChannelNum_t channel_number);
void MCAL_ADC_SetSampleRate(ADC_instance_t ADCx_ID ,ADC_ChannelNum_t channel_number,ADC_SampleTime_t ADC_SamleRate);

#endif /* ADC_H_ */
