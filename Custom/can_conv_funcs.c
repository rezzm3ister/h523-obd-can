#include "can_conv_funcs.h"

int16_t can_ConvPercent(uint32_t in)
{
    return (int16_t)(((float)in /255) * 100 * 100);
}

//no conversion needed for absolute MAP
int16_t can_ConvAbsoluteMAP(uint32_t in)
{
    return in;
}
//no conversion needed for speed
int16_t can_ConvVehicleSpeed(uint32_t in)
{
    return in;
}
//returns actual, multiplier makes it too big
int16_t can_ConvertRPM(uint32_t in)
{
    return (int32_t)((((float)in)/4));
}
//-40 to 210, actual = return/100
int16_t can_ConvTemp1(uint32_t in)
{
    return (int16_t)(((float)in-40)*100);
}
//-40 to 6153, truncated because of range
int16_t can_ConvTemp2(uint32_t in)
{
    return (int16_t)(((((float)in)/10)-40));
}
//-100 to 99.2, actual = return/100, only returns for bank 1
int16_t can_ConvFuelTrim(uint32_t in)
{
    return (int16_t)(((((float)in)*100/128)-100)*100);
}
//unused?
int16_t can_ConvO2Group1(uint32_t in)
{
    return (int16_t)((((float)(in>>16))/200)*100);
}
//o2 sensor, actual lambda = return/1000
int16_t can_ConvO2Group2(uint32_t in)
{
    return (int16_t)((((float)((in>>16)&0xFFFF))*2)/65536*1000);
}
//o2 sensor, actual lambda = return/1000
int16_t can_ConvO2Group3(uint32_t in)
{
    return (int16_t)((((float)((in>>16)&0xFFFF))*2)/65536*1000);
}
//unused
int16_t can_ConvO2Group4(uint32_t in)
{
    return (int16_t)((((((float)in)*100)/128)-100)*100);
}
//degrees before tdc, actual = return/100
int16_t can_ConvTimingAdvance(uint32_t in)
{
    return (int16_t)((((float)in)/2-64)*100);
}

int16_t can_ConvTargetAFR(uint32_t in)
{
    return (int16_t)((((float)in)*2)/65536*100);
}


//mazda specific
//oil temp, actual = return/10
int16_t can_ConvMazdaOilTemp(uint32_t in)
{
    return (int16_t)((((float)in/100)-40)*10);
}

int16_t can_ConvMazdaAtfTemp(uint32_t in)
{
    return (int16_t)((((float)in)/16)*10);
}

//the rest to follow