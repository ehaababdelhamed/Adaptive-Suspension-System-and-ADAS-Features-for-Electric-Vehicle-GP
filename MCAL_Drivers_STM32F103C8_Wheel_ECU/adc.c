/******************************************************************************
 * Module: ADC
 * File Name: ADC.c
 * Description: Source file for ADC driver for Stm32f103x6
 * Author: Khlaid Walid Alazouni
 ******************************************************************************/


/*******************************************************************************
 *                             Includes			                            	*
 *******************************************************************************/
#include "stm32f103x6.h"
#include "ADC.h"

/*******************************************************************************
 *									Macros			                   			*
 *******************************************************************************/
//macro to determine which ADC_instance
#define GET_ADC_BASE_ADD(ADCx_ID) ((ADCx_ID) == ADC1_ID)? (ADC1) : (((ADCx_ID) == ADC2_ID)? ADC2 : NULL)

/*******************************************************************************
 *                            	Function Definition	                   			*
 *******************************************************************************/

/************************************************************************************
 * Function Name	: MCAL_ADC_init
 * Description		: Function to initialize ADC
 * Parameters (in)	: ADCx_ID specifies which ADC used
 * 					  ADCx_Config pointer to ADC configration
 * Return value		: None
 * Note				: Stm32f103x6 supported ADC1,ADC2,ADC3
 * 					  but LQF48 Package supported ADC1,ADC2
 ************************************************************************************/

void MCAL_ADC_init(ADC_instance_t ADCx_ID,ADC_Config_t *ADCx_Config)
{
	boolean error=FALSE;
	ADC_TypeDef *ADCx;
	ADCx = GET_ADC_BASE_ADD(ADCx_ID);
	if(ADCx==NULL)
	{
		error=TRUE;
	}
	if(error==FALSE)
	{
		if(ADCx_ID==ADC1_ID)
		{
			RCC_ADC1_CLK_EN(); //Enable clock for ADC_1
		}
		else
		{
			RCC_ADC2_CLK_EN();//Enable clock for ADC_2
		}

		ADCx->CR2.bit.CONT=ADCx_Config->ADC_ConvMode;          //Select conversion mode single or Continuous in ADC control register 2
		ADCx->CR2.bit.ALIGN=ADCx_Config->ADC_DataAlign;        //Select data alignment right or left in ADC control register 2
		ADCx->SQR1.bit.L=ADCx_Config->ADC_NUM_OF_CAHNNELS;    //Select the number of total channels to be converted in ADC regular sequence register 1
		ADCx->CR2.bit.ADON=TRUE;                              //Enable ADC from ADC control register 2

	}

}

/************************************************************************************
 * Function Name	: MCAL_ADC_deInit
 * Description		: Function to reset all registers of specific ADC.
 * Parameters (in)	: ADCx_ID- Number of Specific instance of ADC @REF: ADC_instance_t
 * Return value		: None
 * Note				: None
 ************************************************************************************/

void MCAL_ADC_Dinit(ADC_instance_t ADCx_ID)
{
	boolean error=FALSE;
	ADC_TypeDef *ADCx;
	ADCx = GET_ADC_BASE_ADD(ADCx_ID);
	if(ADCx==NULL)
	{
		error=TRUE;
	}
	if(error==FALSE)
	{
		if(ADCx_ID==ADC1_ID)
		{
			RCC_ADC1_RESET();               //Disable clock for ADC_1

		}
		else
		{
			RCC_ADC2_RESET();               //Disable clock for ADC_2
		}
		NVIC_IRQ18_ADC1_2_Disable();       //Disable interupts of ADC_1 and ADC_2

	}
}

/************************************************************************************
 * Function Name	: MCAL_ADC_SetSampleRate
 * Description		: Function to define sample_rate for each channel
 * Parameters (in)	: ADCx_ID specifies which ADC used
 * 					  channel_number which channel is used
 * 					  ADC_SamleRate the sample rate for each cahnnel
 * Return value		: None
 * Note				: None
 *
 ************************************************************************************/
void MCAL_ADC_SetSampleRate(ADC_instance_t ADCx_ID ,ADC_ChannelNum_t channel_number,ADC_SampleTime_t ADC_SamleRate)
{

	boolean error=FALSE;
	ADC_TypeDef *ADCx;
	ADCx = GET_ADC_BASE_ADD(ADCx_ID);
	if(ADCx==NULL)
	{
		error=TRUE;
	}
	if(error==FALSE)
	{


		ADCx->SMPR2.ALL_REG|=ADC_SamleRate << (3*channel_number);  //Select the sampling rate for the channel from ADC sample rate register




	}
}

/************************************************************************************
 * Function Name	: MCAL_ADC_Read
 * Description		: Function to read each channel
 * Parameters (in)	: ADCx_ID specifies which ADC used
 * 					  channel_number which channel is used
 * Return value		: The digital data converted from the ADC_channel
 * Note				: None
 *
 ************************************************************************************/

uint32 MCAL_ADC_Read(ADC_instance_t ADCx_ID , ADC_ChannelNum_t channel_number)
{
	uint32 converted_value=FALSE;
	boolean error=FALSE;
	ADC_TypeDef *ADCx;
	ADCx = GET_ADC_BASE_ADD(ADCx_ID);
	if(ADCx==NULL)
	{
		error=TRUE;
	}
	if(error==FALSE)
	{


		ADCx->SQR3.bit.SQ1=channel_number;        //Enter the channel number in ADC regular sequence register 3
		ADCx->CR2.bit.ADON =TRUE;                 //Enable ADC from ADC control register 2
		while(ADCx->SR.bit.EOC==FALSE);
		converted_value = ADCx->DR.bit.DATA;      //read data from ADC regular data register


	}
	return converted_value;

}

