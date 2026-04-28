#ifndef USB_MGR_H
#define USB_MGR_H

#include "usb.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "custom_main.h"

#define MODB_ADDR 0x69
#define MODB_SIZE 0x500
extern uint16_t modb_db[MODB_SIZE];
void USB_CDC_RxHandler(uint8_t* rx_buf_cdc, uint32_t len);

void usb_mainloop(void);


#endif