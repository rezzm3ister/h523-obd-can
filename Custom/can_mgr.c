#include "can_mgr.h"
#include "usb_mgr.h"
#include "fdcan.h"

extern FDCAN_HandleTypeDef hfdcan1;

can_state_t can_state = CAN_INIT; // Initial state of the CAN manager

FDCAN_TxHeaderTypeDef can_tx_header;
uint8_t can_tx_data[8]; // Data buffer for CAN transmission
FDCAN_RxHeaderTypeDef can_rx_header;
uint8_t can_rx_data[8]; // Data buffer for CAN reception
uint8_t can_rx_data_buf[8]; // Data buffer for CAN reception

// target ecu, mode, pid, modb_addr, prio (lower = higher)
uint8_t pid_idx=0;
uint8_t pid_idx_slow=0;

uint8_t pid_max_prio[2]; // 2 prio levels: high/low (0/1)
uint32_t testfunc1(uint32_t val)
{
    return val;
}

can_obd_pid_t can_pids_fast[CAN_PID_COUNT_FAST] =
{
    {.target_addr = 1, .obd_mode = 1, .pid = 1, .raw_value = 5, .conv_value = 5, .conv_func = testfunc1},
    {.target_addr = 2, .obd_mode = 2, .pid = 2, .raw_value = 2, .conv_value = 5, .conv_func = testfunc1}
};
can_obd_pid_t can_pids_slow[CAN_PID_COUNT_SLOW] = 
{
    {.target_addr = 1, .obd_mode = 1, .pid = 1, .raw_value = 5, .conv_value = 5, .conv_func = testfunc1},
};

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
        can_onDataReceived();

    }

    if(can_rx_header.Identifier > 0x7D0)
    {
        
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
        memcpy(can_rx_data_buf,can_rx_data,8);
    }
    can_onDataReceived();

}

void can_onDataReceived(void)
{
    can_sendTestRequest();

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
    // if(pid_table[pid_idx * PID_TABLE_COLS + 3] > 0)
    // {
    //     modb_db[pid_table[pid_idx * PID_TABLE_COLS + 3]] = val;
    // }
    // else
    // {
    //     modb_db[0x100+can_rx_data_buf[2]] = val & 0xFFFF; // Store the processed value in the modbus database
    //     modb_db[0x200+can_rx_data_buf[2]] = val >> 16;
    // }
    // switch(can_state)
    // {
    //     case CAN_PROCESSING:
    //     pid_idx++;
    //     can_state=CAN_WRITE;
    //     if(pid_idx==PID_TABLE_ROWS || (pid_table[pid_idx * PID_TABLE_COLS] < 0x700))
    //     {
    //         pid_idx=0;
    //     }
    //     case CAN_PROCESSING_SLOW:
    //     pid_idx_slow++;
    //     can_state=CAN_WRITE;
    //     if(pid_idx_slow==PID_TABLE_ROWS || (pid_table_slow[pid_idx_slow * PID_TABLE_COLS] < 0x700))
    //     {
    //         pid_idx_slow=0;
    //     }
    //     break;
    //     default:
    //         break;
    // }
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
    can_time++;
    if(can_time == 0xFFFFFFFF)
    {
        can_time = 0;
    }
}

void can_1sloop(void)
{
    // can_sendTestRequest();
}

void can_mainloop(void)
{
    can_pids_fast[0].conv_func(1);
    can_pids_slow[0].conv_func(1);
}
