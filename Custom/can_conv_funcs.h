#ifndef CAN_CONV_FUNCS
#define CAN_CONV_FUNCS
#include "includes.h"

int16_t can_ConvPercent(uint32_t in);
int16_t can_ConvertRPM(uint32_t in);
int16_t can_ConvVehicleSpeed(uint32_t in);
int16_t can_ConvAbsoluteMAP(uint32_t in);
int16_t can_ConvTemp1(uint32_t in);
int16_t can_ConvTemp2(uint32_t in);
int16_t can_ConvFuelTrim(uint32_t in);
int16_t can_ConvO2Group1(uint32_t in);
int16_t can_ConvO2Group2(uint32_t in);
int16_t can_ConvO2Group3(uint32_t in);
int16_t can_ConvO2Group4(uint32_t in);
int16_t can_ConvTimingAdvance(uint32_t in);
int16_t can_ConvTargetAFR(uint32_t in);

//mazda specific
int16_t can_ConvMazdaOilTemp(uint32_t in);
int16_t can_ConvMazdaAtfTemp(uint32_t in);


#endif