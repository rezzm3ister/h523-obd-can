#include "can_mgr.h"
#include "fdcan.h"

extern FDCAN_HandleTypeDef hfdcan1;

FDCAN_TxHeaderTypeDef can_tx_header;
uint8_t can_tx_data[8]; // Data buffer for CAN transmission
FDCAN_RxHeaderTypeDef can_rx_header;
uint8_t can_rx_data[8]; // Data buffer for CAN reception
uint8_t can_rx_data_buf[8]; // Data buffer for CAN reception

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
    can_sendTestRequest();
}

void can_mainloop(void)
{

}