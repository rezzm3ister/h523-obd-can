#include "adxl_mgr.h"
#include "i2c.h"
#include "usb_mgr.h"
extern I2C_HandleTypeDef hi2c3;

adxl_state_t adxl_state = ADXL_OFF;

uint8_t adxl_rx_buffer[ADXL_BUFFER_SIZE];
uint8_t adxl_tx_buffer[ADXL_BUFFER_SIZE];


adxl_init_state_t adxl_init_state = ADXL_INIT_RST;
bool adxl_alive = 0;
bool adxl_isBusy = 0;
static uint32_t adxl_timeouts = 0;
//timers
static uint32_t adxl_init_timer = 0;
static uint32_t adxl_timeout_timer = 0;
static uint32_t adxl_poll_timer = 0;

//data
volatile static int16_t raw_x;
volatile static int16_t raw_y;
volatile static int16_t raw_z;

volatile static float float_x;
volatile static float float_y;
volatile static float float_z;

volatile static float total_g;

void adxl_MemRxCpltCallback(void)
{
    switch(adxl_state)
    {
        case ADXL_OFF:
            if(adxl_rx_buffer[0] == 0xE5)
            {
                adxl_alive = 1;
            }
        break;
        case ADXL_RX:
            raw_x = (adxl_rx_buffer[1]<<8) | adxl_rx_buffer[0];
            raw_y = (adxl_rx_buffer[3]<<8) | adxl_rx_buffer[2];
            raw_z = (adxl_rx_buffer[5]<<8) | adxl_rx_buffer[4];
            float_x = raw_x * 0.0078;
            float_y = raw_y * 0.0078;
            float_z = raw_z * 0.0078;
            total_g = sqrt((float_x*float_x)+(float_y*float_y)+(float_z*float_z));
            modb_db[0x105] = (int16_t)(float_x * 1000);
            modb_db[0x106] = (int16_t)(float_y * 1000);
            modb_db[0x107] = (int16_t)(float_z * 1000);
            modb_db[0x108] = (int16_t)(total_g * 1000);

            adxl_state = ADXL_TX;
            adxl_isBusy = 0;

        break;
        default:
        break;
    }
    adxl_isBusy = 0;

}
void adxl_MemTxCpltCallback(void)
{
    if(adxl_state != ADXL_INIT_TX)
    {
        //only care about init for now
        return;
    }
    switch(adxl_init_state)
    {
        case ADXL_INIT_RST:
            adxl_init_state = ADXL_INIT_WAKE;
        break;
        case ADXL_INIT_WAKE:
            adxl_init_state = ADXL_INIT_SETRANGE;
        break;
        case ADXL_INIT_SETRANGE:
            adxl_init_state = ADXL_INIT_SETSPEED;
        break;
        case ADXL_INIT_SETSPEED:
            adxl_init_state = ADXL_INIT_DONE;
        break;
        default:
        break;
    }
    adxl_isBusy = 0;
}

void adxl_startRead(uint8_t reg, uint8_t len)
{
    HAL_I2C_Mem_Read_IT(&hi2c3,ADXL_ADDR, reg, I2C_MEMADD_SIZE_8BIT, adxl_rx_buffer, len);
}
void adxl_startWrite(uint8_t reg, uint8_t data)
{
    adxl_tx_buffer[0] = data;
    HAL_I2C_Mem_Write_IT(&hi2c3,ADXL_ADDR,reg,I2C_MEMADD_SIZE_8BIT,adxl_tx_buffer,1);
}

void adxl_init(void)
{
    adxl_startRead(0,1);
    adxl_init_state = ADXL_INIT_RST;
    adxl_init_timer = Get10kTick();
    adxl_poll_timer = Get10kTick();
}   

void adxl_mainloop(void)
{
    if(Get10kTick() - adxl_poll_timer < ADXL_TIMER_INTERVAL)
    {
        return;
    }
    switch(adxl_state)
    {
        case ADXL_OFF:
            if(adxl_alive)
            {
                adxl_state = ADXL_INIT_TX;
                adxl_init_state = ADXL_INIT_RST;

            }
            if((Get10kTick() - adxl_init_timer) > 10000)
            {
                adxl_init();
            }
        break;
        case ADXL_INIT_TX:
            switch(adxl_init_state)
            {
                case ADXL_INIT_RST:
                    adxl_isBusy = 1;
                    adxl_startWrite(0x2d,0);
                break;
                case ADXL_INIT_WAKE:
                    adxl_isBusy = 1;
                    adxl_startWrite(0x2d,0x8);
                break;
                case ADXL_INIT_SETRANGE:
                    adxl_isBusy = 1;
                    adxl_startWrite(0x31,0x01);
                break;
                case ADXL_INIT_SETSPEED:
                    adxl_isBusy = 1;
                    adxl_startWrite(0x2C,0b1110);
                break;
                case ADXL_INIT_DONE:
                    adxl_state = ADXL_TX;
                default:
                break;
            }
        break;
        case ADXL_TX:
            adxl_timeout_timer = Get10kTick();
            adxl_isBusy = 1;
            adxl_startRead(0x32,0x6);
            adxl_state = ADXL_RX;
        break;
        case ADXL_RX:
            if(Get10kTick() - adxl_timeout_timer > 5000)
            {
                adxl_timeouts++;
                adxl_state = ADXL_TX;
            }
        break;
        default:
        break;
    }
    adxl_poll_timer = Get10kTick();

}
