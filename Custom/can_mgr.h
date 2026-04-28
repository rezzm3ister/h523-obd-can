#ifndef CAN_MGR_H
#define CAN_MGR_H
#include "main.h"
#include "includes.h"

#define CAN_WAKEUP_TIME 30000
#define CAN_TIMEOUT 150
#define CAN_STARTUP_TIMEOUT 1000
#define CAN_MAX_PID 0x60
#define CAN_PID_COUNT_FAST 5
#define CAN_PID_COUNT_SLOW 5

typedef enum
{
    CAN_INIT=0,
    CAN_TURNON,
    CAN_WRITE,
    CAN_WAIT_RSP,
    CAN_PROCESSING,
    CAN_WRITE_SLOW,
    CAN_WAIT_RSP_SLOW,
    CAN_PROCESSING_SLOW,
    CAN_OFF
} can_state_t;

typedef struct 
{
    uint16_t target_addr;
    uint8_t obd_mode;
    uint8_t pid;
    uint32_t raw_value;
    uint16_t conv_value;
    uint16_t (*conv_func)(uint32_t);

} can_obd_pid_t;


#define PID_TABLE_ROWS 23
#define PID_TABLE_COLS 4
#define FAST_PID_COUNT 17
void can_sendTestRequest(void);
void can_onDataReceived(void);

void can_Init(void);
void can_timingloop(void);
void can_1sloop(void);
void can_mainloop(void);

#endif