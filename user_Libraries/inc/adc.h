#ifndef ADC_H
#define ADC_H   


void ADC_Config(void);
u16 readADC(ADC_TypeDef* ADCx, u8 channel, uint8_t sampleTime);

#define read_Vol_Meter     readADC(ADC1, 3, ADC_SampleTime_84Cycles)
#define read_LF_Sensor     readADC(ADC1, 10,  ADC_SampleTime_84Cycles)
#define read_RF_Sensor     readADC(ADC1, 14, ADC_SampleTime_84Cycles)
#define	read_LD_Sensor     readADC(ADC1, 11,  ADC_SampleTime_84Cycles)
#define read_RD_Sensor     readADC(ADC1, 15, ADC_SampleTime_84Cycles)
#define	read_Outz	       	 readADC(ADC1, 8,  ADC_SampleTime_84Cycles)
#define	read_Vref	         readADC(ADC1, 9,  ADC_SampleTime_84Cycles)
#define read_Rate					 readADC(ADC1, 7, ADC_SampleTime_84Cycles)

#endif
