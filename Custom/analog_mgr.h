#ifndef ANALOG_MGR_H
#define ANALOG_MGR_H
#include "main.h"
#include "includes.h"
#include "adc.h"

#define NUM_ADCS 5

void analog_init(void);
void analog_mainloop(void);
#endif