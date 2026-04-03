#include "usb_mgr.h"
#include "lib_modbus.h"

uint8_t TxBuffer[] = "Hello World! From STM32 USB CDC Device To Virtual COM Port\r\n";
uint8_t TxBufferLen = sizeof(TxBuffer);
uint8_t rx_buf[300];
uint16_t rx_len;
uint8_t tx_buf[300];
bool has_rx=0;

uint16_t modb_db[0x100];

void usb_OnDataReceived(void)
{
    if(rx_buf[0]!=MODB_ADDR) return;

    //add crc check here
    uint16_t reg_addr = (rx_buf[2]<<8) | rx_buf[3];
    uint16_t payload_len = (rx_buf[4]<<8) | rx_buf[5];
    uint8_t r_buf[300];
    uint16_t rsp_len = 3 + payload_len*2;
    // memcpy(w_payload,rx_buf,(payload_len*2));
    tx_buf[0]=MODB_ADDR;
    tx_buf[1]=rx_buf[1];
    // uart_testwrite();
    //do r/w functions
    switch(rx_buf[1])
    {
        case MODB_READ:
        case MODB_READ2:
            modbus_read(&modb_db[reg_addr],r_buf,payload_len);
            // HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,0);
            tx_buf[2]=payload_len*2;
            memcpy(&tx_buf[3],r_buf,payload_len*2);

            break;
        case MODB_WRITE:
            memcpy(&tx_buf[0],&rx_buf[0],6);
            modbus_write(&modb_db[reg_addr],&rx_buf[4],1);
            rsp_len=6;
            break;
        case MODB_WRITEMULT:
            // memcpy(w_payload,&rx_buf[7],rx_buf[6]);
            modbus_write(&modb_db[reg_addr],&rx_buf[7],payload_len);
            memcpy(&tx_buf[2],&rx_buf[2],4);
            rsp_len=6;
            break;
        default:
            break;
    }
    
    uint16_t crc=0;

    crc=crc_modbus(tx_buf,rsp_len);
    tx_buf[rsp_len]=crc&0xff;
    tx_buf[rsp_len+1]=crc>>8;
    // HAL_UART_Transmit_DMA(&huart1,tx_buf,rsp_len+2);
    CDC_Transmit_FS(tx_buf, rsp_len+2);
}

void USB_CDC_RxHandler(uint8_t* rx_buf_cdc, uint32_t len)
{
    // CDC_Transmit_FS(rx_buf_cdc, len);
    // CDC_Transmit_FS(TxBuffer, TxBufferLen);

   memcpy(rx_buf,rx_buf_cdc,len);
   rx_len = len;
   has_rx=1;
}

void usb_mainloop(void)
{
    if(has_rx)
    {
        has_rx=0;
        usb_OnDataReceived();
    }
}
