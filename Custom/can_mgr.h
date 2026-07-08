#ifndef CAN_MGR_H
#define CAN_MGR_H
#include "main.h"
#include "includes.h"

#define CAN_WAKEUP_TIME 30000
#define CAN_TIMEOUT 150
#define CAN_STARTUP_TIMEOUT 1000
#define CAN_MAX_PID 0x60
#define CAN_PID_COUNT_FAST 11
#define CAN_INTERMESSAGE_TIME 1

#ifdef MAZDA
#define CAN_PID_COUNT_SLOW 5
#else
#define CAN_PID_COUNT_SLOW 4
#endif

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
    uint16_t pid;
    uint32_t raw_value;
    int16_t conv_value;
    int16_t (*conv_func)(uint32_t);
    bool is_special;
    uint8_t special_addr;
    uint32_t conv_multiplier;

} can_obd_pid_t;

typedef enum
{
    IDX_ENGINE_LOAD = 0,
    IDX_SHORT_TERM_FUEL_TRIM,
    IDX_MAP,
    IDX_RPM,
    IDX_SPEED,
    IDX_TIMING_ADV,
    IDX_TPS,
    IDX_AFR,
    IDX_EGT,
    IDX_AFR_TGT,
    IDX_TPS_2

} obd_pid_fast_idx_t;
typedef enum
{
    IDX_CLT,
    IDX_IAT,
    IDX_FUEL_LVL,
    #ifdef MAZDA
    IDX_MAZDA_OIL_TEMP,
    IDX_MAZDA_ATF_TEMP,
    #else
    IDX_OIL_TEMP
    #endif
} obd_pid_slow_idx_t;


#define PID_TABLE_ROWS 23
#define PID_TABLE_COLS 4
#define FAST_PID_COUNT 17

uint32_t can_GetSlowTimeouts(void);
uint32_t can_GetFastTimeouts(void);
uint32_t can_GetLoopTime(void);

void can_sendTestRequest(void);
void can_onDataReceived(can_obd_pid_t *h);

uint32_t can_GetRawLoad(void);
int16_t can_GetActualLoad(void);
uint32_t can_GetLoadMultiplier(void);
uint32_t can_GetRawSTFT(void);
uint32_t can_GetSTFTMultiplier(void);
int16_t can_GetActualSTFT(void);
uint32_t can_GetRawMAP(void);
uint32_t can_GetMAPMultiplier(void);
int16_t can_GetActualMAP(void);
uint32_t can_GetRawRPM(void);
uint32_t can_GetRPMMultiplier(void);
int16_t can_GetActualRPM(void);
uint32_t can_GetRawSpeed(void);
uint32_t can_GetSpeedMultiplier(void);
int16_t can_GetActualSpeed(void);
uint32_t can_GetRawIgnAdv(void);
uint32_t can_GetIgnAdvMultiplier(void);
int16_t can_GetActualIgnAdv(void);
uint32_t can_GetRawTPS(void);
uint32_t can_GetTPSMultiplier(void);
int16_t can_GetActualTPS(void);
uint32_t can_GetRawAFR(void);
uint32_t can_GetAFRMultiplier(void);
int16_t can_GetActualAFR(void);
uint32_t can_GetRawEGT(void);
uint32_t can_GetEGTMultiplier(void);
int16_t can_GetActualEGT(void);
uint32_t can_GetRawAFRTGT(void);
uint32_t can_GetAFRTGTMultiplier(void);
int16_t can_GetActualAFRTGT(void);
uint32_t can_GetRawTPS2(void);
uint32_t can_GetTPS2Multiplier(void);
int16_t can_GetActualTPS2(void);
uint32_t can_GetRawCLT(void);
uint32_t can_GetCLTMultiplier(void);
int16_t can_GetActualCLT(void);
uint32_t can_GetRawIAT(void);
uint32_t can_GetIATMultiplier(void);
int16_t can_GetActualIAT(void);
uint32_t can_GetRawFuelLevel(void);
uint32_t can_GetFuelLevelMultiplier(void);
int16_t can_GetActualFuelLevel(void);
uint32_t can_GetRawOilTemp(void);
uint32_t can_GetOilTempMultiplier(void);
int16_t can_GetActualOilTemp(void);
uint32_t can_GetRawAtfTemp(void);
uint32_t can_GetAtfTempMultiplier(void);
int16_t can_GetActualAtfTemp(void);


void can_Init(void);
void can_timingloop(void);
void can_1sloop(void);
void can_mainloop(void);

#endif