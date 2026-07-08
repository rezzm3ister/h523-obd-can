#include "can_mgr.h"
#include "usb_mgr.h"
#include "fdcan.h"
#include "can_conv_funcs.h"

extern FDCAN_HandleTypeDef hfdcan1;

can_state_t can_state = CAN_INIT; // Initial state of the CAN manager

FDCAN_TxHeaderTypeDef can_tx_header;
FDCAN_RxHeaderTypeDef can_rx_header;
uint8_t can_tx_data[8]; // Data buffer for CAN transmission
uint8_t can_rx_data[8]; // Data buffer for CAN reception
uint8_t can_rx_data_buf[8]; // Data buffer for CAN reception

// target ecu, mode, pid, modb_addr, prio (lower = higher)
uint8_t pid_idx=0;
uint8_t pid_idx_slow=0;

bool has_data = false;
//loop durations
static uint32_t can_loop_time = 0;

static uint32_t can_intermessage_timer = 0;

uint8_t pid_max_prio[2]; // 2 prio levels: high/low (0/1)
uint32_t testfunc1(uint32_t val)
{
    return val;
}

volatile can_obd_pid_t can_pids_fast[CAN_PID_COUNT_FAST] =
{
    {.target_addr = 0x7DF, .obd_mode = 1, .pid = 0x04, .conv_func = can_ConvPercent, .conv_multiplier = 100}, // fuel trim
    {.target_addr = 0x7DF, .obd_mode = 1, .pid = 0x06, .conv_func = can_ConvFuelTrim, .conv_multiplier = 100}, // fuel trim
    {.target_addr = 0x7DF, .obd_mode = 1, .pid = 0x0B, .conv_func = can_ConvAbsoluteMAP, .conv_multiplier = 1},
    {.target_addr = 0x7DF, .obd_mode = 1, .pid = 0x0C, .conv_func = can_ConvertRPM, .conv_multiplier = 1},
    {.target_addr = 0x7DF, .obd_mode = 1, .pid = 0x0D, .conv_func = can_ConvVehicleSpeed, .conv_multiplier = 1},
    {.target_addr = 0x7DF, .obd_mode = 1, .pid = 0x0E, .conv_func = can_ConvTimingAdvance, .conv_multiplier = 100},
    {.target_addr = 0x7DF, .obd_mode = 1, .pid = 0x11, .conv_func = can_ConvPercent, .conv_multiplier = 100}, //throttle position
    {.target_addr = 0x7DF, .obd_mode = 1, .pid = 0x34, .conv_func = can_ConvO2Group3, .conv_multiplier = 100}, //Lambda
    {.target_addr = 0x7DF, .obd_mode = 1, .pid = 0x3C, .conv_func = can_ConvTemp2, .conv_multiplier = 1}, //EGT
    {.target_addr = 0x7DF, .obd_mode = 1, .pid = 0x44, .conv_func = can_ConvTargetAFR, .conv_multiplier = 100}, //target AFR
    {.target_addr = 0x7DF, .obd_mode = 1, .pid = 0x45, .conv_func = can_ConvPercent, .conv_multiplier = 100}, //throttle%
};
volatile can_obd_pid_t can_pids_slow[CAN_PID_COUNT_SLOW] = 
{
    {.target_addr = 0x7DF, .obd_mode = 1, .pid = 0x05, .conv_func = can_ConvTemp1, .conv_multiplier = 100},
    {.target_addr = 0x7DF, .obd_mode = 1, .pid = 0x0F, .conv_func = can_ConvTemp1, .conv_multiplier = 100},
    {.target_addr = 0x7DF, .obd_mode = 1, .pid = 0x2F, .conv_func = can_ConvPercent, .conv_multiplier = 100},
    #ifdef MAZDA
    {.target_addr = 0x7E0, .obd_mode = 0x22, .pid = 0x1310, .conv_func = can_ConvMazdaOilTemp, .is_special = 1, .special_addr =0x10, .conv_multiplier = 10},
    {.target_addr = 0x7E1, .obd_mode = 0x22, .pid = 0x1E1C, .conv_func = can_ConvMazdaAtfTemp, .is_special = 1, .special_addr =0x11, .conv_multiplier = 10},
    #else
    {.target_addr = 0x7DF, .obd_mode = 1, .pid = 0x5C, .conv_func = can_ConvTemp1},

    #endif
};


//timers
static uint32_t can_startup_timer = 0;
static uint32_t can_write_timeout_timer = 0;

static uint32_t can_fast_timeouts = 0;
static uint32_t can_slow_timeouts = 0;

static uint32_t last_loop_start = 0;

uint32_t can_GetSlowTimeouts(void)
{
    return can_slow_timeouts;
}
uint32_t can_GetFastTimeouts(void)
{
    return can_fast_timeouts;
}

uint32_t can_GetLoopTime(void)
{
    return can_loop_time;
}

// data getters
uint32_t can_GetRawLoad(void)
{
    return can_pids_fast[IDX_ENGINE_LOAD].raw_value;
}
uint32_t can_GetLoadMultiplier(void)
{
    return can_pids_fast[IDX_ENGINE_LOAD].conv_multiplier;
}
int16_t can_GetActualLoad(void)
{
    return can_pids_fast[IDX_ENGINE_LOAD].conv_value;
}
uint32_t can_GetRawSTFT(void)
{
    return can_pids_fast[IDX_SHORT_TERM_FUEL_TRIM].raw_value;
}
uint32_t can_GetSTFTMultiplier(void)
{
    return can_pids_fast[IDX_SHORT_TERM_FUEL_TRIM].conv_multiplier;
}
int16_t can_GetActualSTFT(void)
{
    return can_pids_fast[IDX_SHORT_TERM_FUEL_TRIM].conv_value;
}
uint32_t can_GetRawMAP(void)
{
    return can_pids_fast[IDX_MAP].raw_value;
}
uint32_t can_GetMAPMultiplier(void)
{
    return can_pids_fast[IDX_MAP].conv_multiplier;
}
int16_t can_GetActualMAP(void)
{
    return can_pids_fast[IDX_MAP].conv_value;
}
uint32_t can_GetRawRPM(void)
{
    return can_pids_fast[IDX_RPM].raw_value;
}
uint32_t can_GetRPMMultiplier(void)
{
    return can_pids_fast[IDX_RPM].conv_multiplier;
}
int16_t can_GetActualRPM(void)
{
    return can_pids_fast[IDX_RPM].conv_value;
}
uint32_t can_GetRawSpeed(void)
{
    return can_pids_fast[IDX_SPEED].raw_value;
}
uint32_t can_GetSpeedMultiplier(void)
{
    return can_pids_fast[IDX_SPEED].conv_multiplier;
}
int16_t can_GetActualSpeed(void)
{
    return can_pids_fast[IDX_SPEED].conv_value;
}
uint32_t can_GetRawIgnAdv(void)
{
    return can_pids_fast[IDX_TIMING_ADV].raw_value;
}
uint32_t can_GetIgnAdvMultiplier(void)
{
    return can_pids_fast[IDX_TIMING_ADV].conv_multiplier;
}
int16_t can_GetActualIgnAdv(void)
{
    return can_pids_fast[IDX_TIMING_ADV].conv_value;
}
uint32_t can_GetRawTPS(void)
{
    return can_pids_fast[IDX_TPS].raw_value;
}
uint32_t can_GetTPSMultiplier(void)
{
    return can_pids_fast[IDX_TPS].conv_multiplier;
}
int16_t can_GetActualTPS(void)
{
    return can_pids_fast[IDX_TPS].conv_value;
}
uint32_t can_GetRawAFR(void)
{
    return can_pids_fast[IDX_AFR].raw_value;
}
uint32_t can_GetAFRMultiplier(void)
{
    return can_pids_fast[IDX_AFR].conv_multiplier;
}
int16_t can_GetActualAFR(void)
{
    return can_pids_fast[IDX_AFR].conv_value;
}
uint32_t can_GetRawEGT(void)
{
    return can_pids_fast[IDX_EGT].raw_value;
}
uint32_t can_GetEGTMultiplier(void)
{
    return can_pids_fast[IDX_EGT].conv_multiplier;
}
int16_t can_GetActualEGT(void)
{
    return can_pids_fast[IDX_EGT].conv_value;
}
uint32_t can_GetRawAFRTGT(void)
{
    return can_pids_fast[IDX_AFR_TGT].raw_value;
}
uint32_t can_GetAFRTGTMultiplier(void)
{
    return can_pids_fast[IDX_AFR_TGT].conv_multiplier;
}
int16_t can_GetActualAFRTGT(void)
{
    return can_pids_fast[IDX_AFR_TGT].conv_value;
}
uint32_t can_GetRawTPS2(void)
{
    return can_pids_fast[IDX_TPS_2].raw_value;
}
uint32_t can_GetTPS2Multiplier(void)
{
    return can_pids_fast[IDX_TPS_2].conv_multiplier;
}
int16_t can_GetActualTPS2(void)
{
    return can_pids_fast[IDX_TPS_2].conv_value;
}
uint32_t can_GetRawCLT(void)
{
    return can_pids_slow[IDX_CLT].raw_value;
}
uint32_t can_GetCLTMultiplier(void)
{
    return can_pids_slow[IDX_CLT].conv_multiplier;
}
int16_t can_GetActualCLT(void)
{
    return can_pids_slow[IDX_CLT].conv_value;
}
uint32_t can_GetRawIAT(void)
{
    return can_pids_slow[IDX_IAT].raw_value;
}
uint32_t can_GetIATMultiplier(void)
{
    return can_pids_slow[IDX_IAT].conv_multiplier;
}
int16_t can_GetActualIAT(void)
{
    return can_pids_slow[IDX_IAT].conv_value;
}
uint32_t can_GetRawFuelLevel(void)
{
    return can_pids_slow[IDX_FUEL_LVL].raw_value;
}
uint32_t can_GetFuelLevelMultiplier(void)
{
    return can_pids_slow[IDX_FUEL_LVL].conv_multiplier;
}
int16_t can_GetActualFuelLevel(void)
{
    return can_pids_slow[IDX_FUEL_LVL].conv_value;
}
#ifdef MAZDA
uint32_t can_GetRawOilTemp(void)
{
    return can_pids_slow[IDX_MAZDA_OIL_TEMP].raw_value;
}
uint32_t can_GetOilTempMultiplier(void)
{
    return can_pids_slow[IDX_MAZDA_OIL_TEMP].conv_multiplier;
}
int16_t can_GetActualOilTemp(void)
{
    return can_pids_slow[IDX_MAZDA_OIL_TEMP].conv_value;
}
uint32_t can_GetRawAtfTemp(void)
{
    return can_pids_slow[IDX_MAZDA_ATF_TEMP].raw_value;
}
uint32_t can_GetAtfTempMultiplier(void)
{
    return can_pids_slow[IDX_MAZDA_ATF_TEMP].conv_multiplier;
}
int16_t can_GetActualAtfTemp(void)
{
    return can_pids_slow[IDX_MAZDA_ATF_TEMP].conv_value;
}
#else
uint32_t can_GetRawOilTemp(void)
{
    return can_pids_slow[IDX_OIL_TEMP].raw_value;
}
uint32_t can_GetOilTempMultiplier(void)
{
    return can_pids_slow[IDX_OIL_TEMP].conv_multiplier;
}
int16_t can_GetActualOilTemp(void)
{
    return can_pids_slow[IDX_OIL_TEMP].conv_value;
}

#endif




void can_sendTestRequest(void)
{
    can_tx_header.Identifier = 0x7DF; // Standard ID for OBD-II requests
    can_tx_header.IdType = FDCAN_STANDARD_ID;
    // can_tx_header.TxFrameType = FDCAN_REMOTE_FRAME;
    can_tx_header.TxFrameType = FDCAN_DATA_FRAME;
    can_tx_header.DataLength = FDCAN_DLC_BYTES_8; // OBD-II requests typically use 8 bytes
    can_tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    can_tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    can_tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    can_tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // No Tx event FIFO control
    can_tx_header.MessageMarker = 0; // Not used in this context

    memset(can_tx_data, 0xFF, 8); // Clear the data buffer

    // can_tx_data[0]=2;
    can_tx_data[1]=0x69; // OBD-II request
    // can_tx_data[2]=target_pid; // PID to request
    if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &can_tx_header, can_tx_data) !=0)
    {
        Error_Handler();
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        /* Retreive Rx messages from RX FIFO0 */
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &can_rx_header, can_rx_data) != HAL_OK)
        {
        /* Reception Error */
        Error_Handler();
        }

        if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
        {
        /* Notification Error */
        Error_Handler();
        }
        // can_onDataReceived();

    }
    memcpy(can_rx_data_buf,can_rx_data,8);
    has_data = true;
    // switch(can_state)
    // {
    //     case CAN_WAIT_RSP:
    //         can_state = CAN_PROCESSING; // Set state to processing after receiving data

    //     break;
    //     case CAN_WAIT_RSP_SLOW:
    //         can_state = CAN_PROCESSING_SLOW; // Set state to processing after receiving data

    //     break;
    //     default:
    //     return;
    //     break;
    // }
    

}

void can_sendRequest(can_obd_pid_t *h)
{
    can_tx_header.Identifier = h->target_addr; // Standard ID for OBD-II requests
    can_tx_header.IdType = FDCAN_STANDARD_ID;
    // can_tx_header.TxFrameType = FDCAN_REMOTE_FRAME;
    can_tx_header.TxFrameType = FDCAN_DATA_FRAME;
    can_tx_header.DataLength = FDCAN_DLC_BYTES_8; // OBD-II requests typically use 8 bytes
    can_tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    can_tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    can_tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    can_tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // No Tx event FIFO control
    can_tx_header.MessageMarker = 0; // Not used in this context

    memset(can_tx_data, 0xFF, 8);
    can_tx_data[1] = h->obd_mode;

    if(h->pid > 0xFF)
    {
        can_tx_data[0] = 3;
        can_tx_data[2] = h->pid >> 8;
        can_tx_data[3] = h->pid & 0xFF;
    }
    else
    {
        can_tx_data[0] = 2;
        can_tx_data[2] = h->pid & 0xFF;
    }
    if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &can_tx_header, can_tx_data) !=0)
    {
        Error_Handler();
    }
}



void can_onDataReceived(can_obd_pid_t *h)
{
    // can_sendTestRequest();

    volatile uint32_t val = 0;
    uint8_t bytes_following = 0;
    uint8_t len = 0;
    uint8_t data[4];
    bool bad = 0;
    //checks if its mode 1 or mode 22
    if(can_rx_data_buf[1] != 0x41)
    {
        // if(can_rx_data_buf[2] != h->pid)
        // {
        //     return;
        // }
        len = can_rx_data_buf[0]-3;
        memcpy(data, &can_rx_data_buf[4], 4);
    }
    else
    {
        // if((can_rx_data_buf[2]<<8 | can_rx_data_buf[3]) != h->pid)
        // {
        //     return;
        // }
        len = can_rx_data_buf[0]-2;
        memcpy(data, &can_rx_data_buf[3], 4);
    }

    switch(len)
    {
        case 1:
            val = data[0];
            break;
        case 2:
            val = (data[0] << 8) | data[1];
            break;
        case 3:
            val = (data[0] << 16) | (data[1] << 8) | data[2];
            break;
        case 4:
            val = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
            break;
        default:
            // Handle unexpected length
            val = 0;
            can_intermessage_timer = Get10kTick();
            return;
            break;
    }
    h->raw_value = val;
    h->conv_value = h->conv_func(val);
    if(h->is_special)
    {
        modb_db[0x100 + h->special_addr] = h->conv_value;
    }
    else
    {
        modb_db[h->pid] = h->conv_value;
    }
    can_intermessage_timer = Get10kTick();
}

void can_Init(void)
{
    if(HAL_FDCAN_Start(&hfdcan1)!= HAL_OK)
    {
        Error_Handler();
    }
    // Initialize CAN hardware and configure filters
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        /* Notification Error */
        Error_Handler();
    }
}
uint32_t can_time = 0;
void can_timingloop(void)
{
    // can_time++;
    // if(can_time == 0xFFFFFFFF)
    // {
    //     can_time = 0;
    // }
}

void can_1sloop(void)
{
    // can_sendTestRequest();
}

void can_mainloop(void)
{
    switch(can_state)
    {
        case CAN_INIT:
            can_startup_timer = Get10kTick();
            can_state = CAN_TURNON;
        break;
        case CAN_TURNON:
            if(Get10kTick() - can_startup_timer >= 30000)
            {
                can_state = CAN_WRITE;
            }
        break;
        case CAN_WRITE:
            if(Get10kTick() - can_intermessage_timer > CAN_INTERMESSAGE_TIME)
            {
                can_sendRequest(&can_pids_fast[pid_idx]);
                can_write_timeout_timer = Get10kTick();
                can_state = CAN_WAIT_RSP;
            }
            
        break;
        case CAN_WAIT_RSP:
            if(has_data)
            {
                can_state = CAN_PROCESSING;
                has_data = false;
            }
            else if(Get10kTick() - can_write_timeout_timer > CAN_TIMEOUT)
            {
                // timeout stuff
                pid_idx++;
                can_state = CAN_WRITE;
                if(pid_idx >= CAN_PID_COUNT_FAST)
                {
                    pid_idx = 0;
                    can_state = CAN_WRITE_SLOW;
                }
                can_fast_timeouts++;
            }
        break;
        case CAN_PROCESSING:
            can_onDataReceived(&can_pids_fast[pid_idx]);
            pid_idx++;
            can_state = CAN_WRITE;
            if(pid_idx >= CAN_PID_COUNT_FAST)
            {
                pid_idx = 0;
                can_state = CAN_WRITE_SLOW;
            }
        break;
        case CAN_WRITE_SLOW:
            if(Get10kTick() - can_intermessage_timer > CAN_INTERMESSAGE_TIME)
            {
                can_sendRequest(&can_pids_slow[pid_idx_slow]);
                
                can_loop_time = Get10kTick() - last_loop_start;
                last_loop_start = Get10kTick();
                can_write_timeout_timer = Get10kTick();
                can_state = CAN_WAIT_RSP_SLOW;    
            }
        break;
        case CAN_WAIT_RSP_SLOW:
            if(has_data)
            {
                can_state = CAN_PROCESSING_SLOW;
                has_data = false;
            }
            else if(Get10kTick() - can_write_timeout_timer > CAN_TIMEOUT)
            {
                // timeout stuff
                pid_idx_slow++;
                can_state = CAN_WRITE;
                if(pid_idx_slow >= CAN_PID_COUNT_SLOW)
                {
                    pid_idx_slow = 0;
                }
                can_slow_timeouts++;
            }
        break;
        case CAN_PROCESSING_SLOW:
            can_onDataReceived(&can_pids_slow[pid_idx_slow]);
            pid_idx_slow++;
            if(pid_idx_slow >= CAN_PID_COUNT_SLOW)
            {
                pid_idx_slow = 0;
            }
            can_state = CAN_WRITE;
        break;
        case CAN_OFF:
        break;
        default:
        break;
    }
}
