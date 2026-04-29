#include "custom_main.h"
#include "tim.h"
#include "gpio.h"
#include "usb_mgr.h"
#include "can_mgr.h"
#include "analog_mgr.h"

static uint32_t t1 = 0;
bool one_sec_flag = false;
static uint32_t tick10k = 0;

uint32_t Get10kTick(void)
{
    return tick10k;
}

//10khz timing
void timing_loop(void)
{
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,1);
    tick10k++;
    can_timingloop();
    if(t1<=10000)
    {
        t1++;
    }
    else
    {
        t1 = 0;
        one_sec_flag = true;
    }
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,0);

}

void custom_init(void)
{
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim7);
    MX_USB_Device_Init();
    can_Init();
    analog_init();
    // HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,1);

}

bool led_en = 0;
void mainloop(void)
{
    HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_1);
    if(one_sec_flag)
    {
        one_sec_flag = false;
        if(led_en==1)
            {
                led_en = 0;
            }
            else
            {
                led_en = 1;
            }

        can_1sloop();
    }
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,led_en);
    usb_mainloop();
    can_mainloop();
}
