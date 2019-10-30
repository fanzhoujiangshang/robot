#include "adc1.h"
#include "delay.h"

/********************************************************************************	 
 * ATKflight�ɿع̼�
 * adc1��������	
 * ��������:2018/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * All rights reserved
********************************************************************************/

//volatile uint16_t adcValues;
volatile uint16_t adcValues[2];

void adc1Init(void)
{
	ADC_InitTypeDef       ADC_InitStructure;   
	ADC_CommonInitTypeDef ADC_CommonInitStructure;   
	GPIO_InitTypeDef      GPIO_InitStructure;   

	DMA_InitTypeDef      DMA_InitStructure;
		
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE);	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); 


//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4;//����PA3 ��PA4Ϊ����ͨ��	 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;  
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;   
	GPIO_Init(GPIOA, &GPIO_InitStructure);	 

	DMA_DeInit(DMA2_Stream0);
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;    
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	   DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&adcValues;
	//DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADCvalue;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = 2;	//�趨Ҫת����DMAͨ����Ŀ 
//	DMA_InitStructure.DMA_BufferSize = 1;	//�趨Ҫת����DMAͨ����Ŀ 
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//�ڴ��ַ����ģʽ
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; 
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;  
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; 
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;           
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;  
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;  
	DMA_Init(DMA2_Stream0, &DMA_InitStructure); 
	DMA_Cmd(DMA2_Stream0, ENABLE);     


	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, ENABLE);	  
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, DISABLE);
	
	#if 1
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;   
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_2;  
	ADC_CommonInitStructure.ADC_TwoSamplingDelay=ADC_TwoSamplingDelay_5Cycles;   
	ADC_CommonInit(&ADC_CommonInitStructure);  
	
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;   
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;//����ɨ��ģʽ 
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//��������ת��    
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;											
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;  
	ADC_InitStructure.ADC_NbrOfConversion = 2;//�趨ADҪת����ͨ����Ŀ
//	ADC_InitStructure.ADC_NbrOfConversion = 1;//�趨ADҪת����ͨ����Ŀ
	ADC_Init(ADC1, &ADC_InitStructure);  
	#endif

#if 0
    ADC_CommonStructInit(&ADC_CommonInitStructure);
    ADC_CommonInitStructure.ADC_Mode             = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler        = ADC_Prescaler_Div8;
    ADC_CommonInitStructure.ADC_DMAAccessMode    = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    ADC_StructInit(&ADC_InitStructure);
    ADC_InitStructure.ADC_ContinuousConvMode       = ENABLE;
    ADC_InitStructure.ADC_Resolution               = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ExternalTrigConv         = ADC_ExternalTrigConv_T1_CC1;
    ADC_InitStructure.ADC_ExternalTrigConvEdge     = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign                = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion          = 1;
    ADC_InitStructure.ADC_ScanConvMode             = DISABLE;
    ADC_Init(ADC1, &ADC_InitStructure);
#endif
	
	/*******����ADC1��ͨ��6��ͨ��4��ת���Ⱥ�˳���Լ��������ʱ��Ϊ15����������*******/  
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 1, ADC_SampleTime_15Cycles); 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 2, ADC_SampleTime_15Cycles);   
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_15Cycles); 
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);   
	ADC_DMACmd(ADC1, ENABLE);  
	ADC_Cmd(ADC1, ENABLE);    
	ADC_SoftwareStartConv(ADC1);


}

