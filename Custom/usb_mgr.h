#ifndef USB_MGR_H
#define USB_MGR_H

#include "usb.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "custom_main.h"

#define MODB_ADDR 0x69
#define MODB_SIZE 0x500
extern uint16_t modb_db[MODB_SIZE];

#define CAN_LOOPTIME modb_db[0x200]

/**
 * @brief Check CRC for incoming Modbus request
 * @param rx_buf: Received buffer (must contain at least 8 bytes)
 * @param len: Length of received data
 * @return true if CRC is valid, false otherwise
 */
bool check_crc_modbus(uint8_t *rx_buf, uint16_t len);

/**
 * @brief Get the current Modbus CRC error count
 * @return uint32_t: Current CRC error counter value
 */
uint32_t get_modb_crc_error_count(void);

/**
 * @brief Increment the Modbus CRC error counter
 */
void inc_modb_crc_error_count(void);

void USB_CDC_RxHandler(uint8_t* rx_buf_cdc, uint32_t len);

void usb_mainloop(void);


#endif