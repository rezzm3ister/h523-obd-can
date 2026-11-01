#include "gpio_mgr.h"
#include "lib-timer.h"

bool lcd_mode = 0;
HPTim_t lcd_mode_validation_timer;
gpio_state_t lcd_mode_tfsmv = GPIO_STATE_OFF;
bool gpio_GetLcdMode(void)
{
    return lcd_mode;
}
bool gpio_getRawLcdPin(void)
{
    return(GPIOB->IDR & (1 << 2));
}


void gpio_mainloop()
{
    switch(lcd_mode_tfsmv)
    {
        case GPIO_STATE_OFF:
            lcd_mode = 0;
            if(gpio_getRawLcdPin() != 0)
            {
                lcd_mode_tfsmv = GPIO_STATE_OFF_TO_ON;
                hptim_start(&lcd_mode_validation_timer, LCD_MODE_VALIDATION_TIME);
            }
            break;
        case GPIO_STATE_OFF_TO_ON:
            if(gpio_getRawLcdPin() == 0)
            {
                lcd_mode_tfsmv = GPIO_STATE_OFF;
            }
            else
            {
                hptim_update(&lcd_mode_validation_timer);
                if(HPTIM_IS_EXPIRED(&lcd_mode_validation_timer))
                {
                    lcd_mode_tfsmv = GPIO_STATE_ON;
                }
            }
            break;
        case GPIO_STATE_ON:
            lcd_mode = 1;
            if(gpio_getRawLcdPin() != 1)
            {
                lcd_mode_tfsmv = GPIO_STATE_ON_TO_OFF;
                hptim_start(&lcd_mode_validation_timer, LCD_MODE_VALIDATION_TIME);
            }
            break;
        case GPIO_STATE_ON_TO_OFF:
            if(gpio_getRawLcdPin() == 1)
            {
                lcd_mode_tfsmv = GPIO_STATE_ON;
            }
            else
            {
                hptim_update(&lcd_mode_validation_timer);
                if(HPTIM_IS_EXPIRED(&lcd_mode_validation_timer))
                {
                    lcd_mode_tfsmv = GPIO_STATE_OFF;
                }
            }
            break;
        
    }
}