#include "gpio_mgr.h"

bool lcd_mode = 0;
uint32_t lcd_mode_validation = 0;
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
                lcd_mode_validation = Get10kTick();
            }
            break;
        case GPIO_STATE_OFF_TO_ON:
            if(gpio_getRawLcdPin() == 0)
            {
                lcd_mode_tfsmv = GPIO_STATE_OFF;
            }
            else
            {
                if(Get10kTick() - lcd_mode_validation >= LCD_MODE_VALIDATION_TIME)
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
                lcd_mode_validation = Get10kTick();
            }
            break;
        case GPIO_STATE_ON_TO_OFF:
            if(gpio_getRawLcdPin() == 1)
            {
                lcd_mode_tfsmv = GPIO_STATE_ON;
            }
            else
            {
                if(Get10kTick() - lcd_mode_validation >= LCD_MODE_VALIDATION_TIME)
                {
                    lcd_mode_tfsmv = GPIO_STATE_OFF;
                }
            }
        break;
        
    }
}