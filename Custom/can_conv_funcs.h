#ifndef CAN_CONV_FUNCS
#define CAN_CONV_FUNCS
#include "includes.h"

int16_t can_ConvPercent(const uint32_t in);
int16_t can_ConvertRPM(const uint32_t in);
int16_t can_ConvVehicleSpeed(const uint32_t in);
int16_t can_ConvAbsoluteMAP(const uint32_t in);
int16_t can_ConvTemp1(const uint32_t in);
int16_t can_ConvTemp2(const uint32_t in);
int16_t can_ConvFuelTrim(const uint32_t in);
int16_t can_ConvO2Group1(const uint32_t in);
int16_t can_ConvO2Group2(const uint32_t in);
int16_t can_ConvO2Group3(const uint32_t in);
int16_t can_ConvO2Group4(const uint32_t in);
int16_t can_ConvTimingAdvance(const uint32_t in);
int16_t can_ConvTargetAFR(const uint32_t in);

//mazda specific
int16_t can_ConvMazdaOilTemp(const uint32_t in);
int16_t can_ConvMazdaAtfTemp(const uint32_t in);


#endif