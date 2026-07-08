#include "can_conv_funcs.h"

int16_t can_ConvPercent(const uint32_t in)
{
    return (int16_t)((((float)in )/255) * 100 * 100);
}

//no conversion needed for absolute MAP
int16_t can_ConvAbsoluteMAP(const uint32_t in)
{
    return (int16_t)in;
}
//no conversion needed for speed
int16_t can_ConvVehicleSpeed(const uint32_t in)
{
    return (int16_t)in;
}
//returns actual, multiplier makes it too big
int16_t can_ConvertRPM(const uint32_t in)
{
    return (int32_t)((((float)in)/4));
}
//-40 to 210, actual = return/100
int16_t can_ConvTemp1(const uint32_t in)
{
    return (int16_t)(((float)in-40)*100);
}
//-40 to 6153, truncated because of range
int16_t can_ConvTemp2(const uint32_t in)
{
    return (uint16_t)(((((float)in)/10)-40));
}
//-100 to 99.2, actual = return/100, only returns for bank 1
int16_t can_ConvFuelTrim(const uint32_t in)
{
    return (int16_t)(((((float)in)/1.28)-100)*100);
}
//unused?
int16_t can_ConvO2Group1(const uint32_t in)
{
    return (int16_t)((((float)(in>>16))/200)*100);
}
//o2 sensor, actual lambda = return/1000
int16_t can_ConvO2Group2(const uint32_t in)
{
    return (int16_t)(((((float)((in>>16)&0xFFFF))*2)/65536)*100);
}
//o2 sensor, actual lambda = return/1000
int16_t can_ConvO2Group3(const uint32_t in)
{
    return (int16_t)(((((float)(in>>16))*2)/65536)*100);
    // float temp = (float)((in>>16)&0xFFFF);
    // // temp = temp*2.0/65536*100;
    // // return (int16_t)(((((float)((in>>16)&0xFFFF))*2)/65536)*100);
    // return (int16_t)(temp*2.0/65536*100);
}
//unused
int16_t can_ConvO2Group4(const uint32_t in)
{
    return (int16_t)((((((float)in)*100)/128)-100)*100);
}
//degrees before tdc, actual = return/100
int16_t can_ConvTimingAdvance(const uint32_t in)
{
    return (int16_t)((((float)in)/2.0-64)*100);
}

int16_t can_ConvTargetAFR(const uint32_t in)
{
    return (int16_t)((((float)in)*2)/65536*100);
}


//mazda specific
//oil temp, actual = return/10
int16_t can_ConvMazdaOilTemp(const uint32_t in)
{
    return (int16_t)((((float)in/100)-40)*10);
}

int16_t can_ConvMazdaAtfTemp(const uint32_t in)
{
    return (int16_t)((((float)in)/16)*10);
}

//the rest to follow