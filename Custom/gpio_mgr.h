#ifndef GPIO_MGR_H
#define GPIO_MGR_H
#include "main.h"
#include "includes.h"

#define LCD_MODE_VALIDATION_TIME 1000

typedef enum
{
    GPIO_STATE_OFF = 0,
    GPIO_STATE_OFF_TO_ON,
    GPIO_STATE_ON,
    GPIO_STATE_ON_TO_OFF
}gpio_state_t;

bool gpio_GetLcdMode(void);

void gpio_mainloop();


#endif