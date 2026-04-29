#include "analog_mgr.h"
extern ADC_HandleTypeDef hadc1;

uint16_t adc_data[NUM_ADCS];

void analog_init(void)
{
    HAL_ADC_Start_DMA(&hadc1, &adc_data, NUM_ADCS);
}

void analog_mainloop(void)
{

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

}
