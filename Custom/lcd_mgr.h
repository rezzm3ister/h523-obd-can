#ifndef LCD_MGR_H
#define LCD_MGR_H
#define I2C_QUEUE_BUFFER_SIZE 512

#define LCD_ADDR (0x4E)
#define NUM_LCD_INIT_STEPS 9
#include "i2c.h"
#include "includes.h"

//buffer to copy and paste to reset queue start to 0
typedef struct
{
    uint8_t queue_pos;
    uint8_t queue_end_pos;
    uint8_t b_queue[I2C_QUEUE_BUFFER_SIZE];
    uint8_t queue_pos_data[I2C_QUEUE_BUFFER_SIZE]; // length of each index
    uint8_t queue_pos_addr[I2C_QUEUE_BUFFER_SIZE]; //i2c addr of device to send to for each index
    uint8_t queue_msg_size; //sets max message size per element of queue

    //for isr stuff
    uint8_t tx_send_idx;
    bool tx_ongoing;
    bool prev_tx_ongoing;
}i2c_queue_t;


typedef enum
{
    LCD_STATE_OFF = 0,
    LCD_STATE_INIT,
    LCD_STATE_IDLE,
    LCD_STATE_TX
} lcd_state_t;

// typedef enum
// {

// } lcd_init_state_t;
lcd_state_t lcd_GetLcdState(void);

void lcd_check_alive_rx(void);

void lcd_OnDataTransmit(void);

void lcd_init(void);

void lcd_mainloop(void);

#endif