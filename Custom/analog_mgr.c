#include "analog_mgr.h"
#include "usb_mgr.h"
extern ADC_HandleTypeDef hadc1;

uint16_t adc_data[NUM_ADCS];

void analog_init(void)
{
    HAL_ADC_Start_DMA(&hadc1, adc_data, NUM_ADCS);
}

void analog_mainloop(void)
{
    
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    for(uint8_t i =0; i<NUM_ADCS; i++)
    {
        modb_db[0x100 + i] = adc_data[i];
    }
}
