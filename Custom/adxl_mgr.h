#ifndef ADXL_MGR
#define ADXL_MGR
#include "includes.h"
#define ADXL_ADDR (0x53<<1)

#define ADXL_BUFFER_SIZE 64
#define ADXL_TIMER_INTERVAL 100

typedef enum
{
    ADXL_OFF = 0,
    ADXL_INIT_TX,
    ADXL_INIT_RX,
    ADXL_TX,
    ADXL_RX,

} adxl_state_t;

typedef enum
{
    ADXL_INIT_RST = 0,
    ADXL_INIT_WAKE,
    ADXL_INIT_SETRANGE,
    ADXL_INIT_SETSPEED,
    ADXL_INIT_DONE
} adxl_init_state_t;

void adxl_MemRxCpltCallback(void);

void adxl_MemTxCpltCallback(void);


void adxl_init(void);
void adxl_mainloop(void);




#endif