#include "can_mgr.h"
#include "usb_mgr.h"
#include "fdcan.h"

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

//loop durations
static uint32_t can_loop_time = 0;

uint8_t pid_max_prio[2]; // 2 prio levels: high/low (0/1)
uint32_t testfunc1(uint32_t val)
{
    return val;
}

can_obd_pid_t can_pids_fast[CAN_PID_COUNT_FAST] =
{
    {.target_addr = 0x7DF, .obd_mode = 1, .pid = 1, .conv_func = testfunc1},
    {.target_addr = 0x7DF, .obd_mode = 1, .pid = 2, .conv_func = testfunc1},
    {.target_addr = 0x7DF, .obd_mode = 1, .pid = 3, .conv_func = testfunc1},
    {.target_addr = 0x7DF, .obd_mode = 1, .pid = 4, .conv_func = testfunc1},
    {.target_addr = 0x7DF, .obd_mode = 1, .pid = 5, .conv_func = testfunc1},
};
can_obd_pid_t can_pids_slow[CAN_PID_COUNT_SLOW] = 
{
    {.target_addr = 0x7DF, .obd_mode = 1, .pid = 1, .conv_func = testfunc1},
    {.target_addr = 0x7DF, .obd_mode = 1, .pid = 2, .conv_func = testfunc1},
    {.target_addr = 0x7DF, .obd_mode = 1, .pid = 3, .conv_func = testfunc1},
    {.target_addr = 0x7DF, .obd_mode = 1, .pid = 4, .conv_func = testfunc1},
    {.target_addr = 0x7DF, .obd_mode = 1, .pid = 5, .conv_func = testfunc1},
};


//timers
static uint32_t can_startup_timer = 0;
static uint32_t can_write_timeout_timer = 0;

static uint32_t can_fast_timeouts = 0;
static uint32_t can_slow_timeouts = 0;

static uint32_t last_loop_start = 0;


uint32_t can_GetLoopTime(void)
{
    return can_loop_time;
}


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

    switch(can_state)
    {
        case CAN_WAIT_RSP:
            can_state = CAN_PROCESSING; // Set state to processing after receiving data

        break;
        case CAN_WAIT_RSP_SLOW:
            can_state = CAN_PROCESSING_SLOW; // Set state to processing after receiving data

        break;
        default:
        return;
        break;
    }
    

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



void can_onDataReceived(void)
{
    // can_sendTestRequest();

    volatile uint32_t val = 0;
    uint8_t bytes_following = 0;
    uint8_t len = 0;
    uint8_t data[4];
    bool bad = 0;
    if(can_rx_data_buf[1] != 0x41)
    {
        len = can_rx_data_buf[0]-3;
        memcpy(data, &can_rx_data_buf[4], 4);
    }
    else
    {
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
            break;
    }

    switch(can_state)
    {
        case CAN_PROCESSING:
            // if(pid_table[pid_idx * PID_TABLE_COLS + 3] > 0)
            // {
            //     modb_db[pid_table[pid_idx * PID_TABLE_COLS + 3]] = val;
            // }
            // else
            // {
            //     modb_db[0x100+can_rx_data_buf[2]] = val & 0xFFFF; // Store the processed value in the modbus database
            //     modb_db[0x200+can_rx_data_buf[2]] = val >> 16;
            // }
        break;
        case CAN_PROCESSING_SLOW:
            // if(pid_table_slow[pid_idx_slow * PID_TABLE_COLS + 3] > 0)

            // {
            //     if(val > 0xFF)
            //     modb_db[pid_table_slow[pid_idx_slow * PID_TABLE_COLS + 3]] = val;
            // }
            // else
            // {
            //     modb_db[0x100+can_rx_data_buf[2]] = val & 0xFFFF; // Store the processed value in the modbus database
            //     modb_db[0x200+can_rx_data_buf[2]] = val >> 16;
            // }

        break;
    }
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
            can_sendRequest(&can_pids_fast[pid_idx]);
            can_write_timeout_timer = Get10kTick();
            can_state = CAN_WAIT_RSP;
        break;
        case CAN_WAIT_RSP:
            if(Get10kTick() - can_write_timeout_timer > CAN_TIMEOUT)
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
            can_onDataReceived();
            pid_idx++;
            can_state = CAN_WRITE;
            if(pid_idx >= CAN_PID_COUNT_FAST)
            {
                pid_idx = 0;
                can_state = CAN_WRITE_SLOW;
            }
        break;
        case CAN_WRITE_SLOW:
            can_sendRequest(&can_pids_slow[pid_idx_slow]);

            can_loop_time = Get10kTick() - last_loop_start;
            last_loop_start = Get10kTick();
            can_write_timeout_timer = Get10kTick();
            can_state = CAN_WAIT_RSP_SLOW;
        break;
        case CAN_WAIT_RSP_SLOW:
            if(Get10kTick() - can_write_timeout_timer > CAN_TIMEOUT)
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
            can_onDataReceived();
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
