#include "custom_main.h"
#include "tim.h"
#include "gpio.h"
#include "usb_mgr.h"
#include "can_mgr.h"

uint32_t t1 = 0;
bool one_sec_flag = false;

void timing_loop(void)
{
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
}

void custom_init(void)
{
    HAL_TIM_Base_Start_IT(&htim7);
    MX_USB_Device_Init();
    can_Init();
    // HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,1);

}

bool led_en = 0;
void mainloop(void)
{
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
    HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0);
    usb_mainloop();
}
