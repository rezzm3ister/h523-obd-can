#include "lcd_mgr.h"

extern I2C_HandleTypeDef hi2c2;

i2c_queue_t lcd_queue;
uint8_t lcd_init_step = 0;
lcd_state_t lcd_state = LCD_STATE_OFF;
bool lcd_connected = 0;
static uint8_t lcd_rx_buf[8];
// static uint8_t lcd_rx_buf[8];

static uint8_t lcd_init_commands[NUM_LCD_INIT_STEPS] = 
{
    0x30,
    0x30,
    0x30,
    0x20,
    0x28,
    0x08,
    0x01,
    0x06,
    0x0C
};

static uint8_t lcd_init_command_delays[NUM_LCD_INIT_STEPS] = 
{
    500,
    500,
    10,
    100,
    100,
    50,
    50,
    100,
    50,
};

//timers
static uint32_t lcd_turnon_timer = 0;
static uint32_t lcd_init_func_timer = 0;


lcd_state_t lcd_GetLcdState(void)
{
    return lcd_state;
}


void lcd_queue_init(i2c_queue_t * q, uint8_t size)
{
    memset(q->b_queue,0,I2C_QUEUE_BUFFER_SIZE);
    memset(q->queue_pos_data,0,I2C_QUEUE_BUFFER_SIZE);
    q->queue_end_pos = 0;
    q->queue_pos = 0;
    q->queue_msg_size = size;
    q->tx_ongoing = 0;
    //enable interrupts
}
void i2c_queue_clear(i2c_queue_t *q)
{
    memset(q->b_queue,0,I2C_QUEUE_BUFFER_SIZE);
    memset(q->queue_pos_data,0,I2C_QUEUE_BUFFER_SIZE);
    q->queue_end_pos = 0;
    q->queue_pos = 0;
    q->tx_send_idx = 0;
}
void lcd_queue_append(i2c_queue_t * q, uint8_t addr, uint8_t * data, uint8_t len)
{
    memcpy(&q->b_queue[q->queue_end_pos * q->queue_msg_size],data,len);
    q->queue_pos_addr[q->queue_end_pos] = addr;
    q->queue_pos_data[q->queue_end_pos] = len;

    q->queue_end_pos++;
}
void i2c_advance_queue(i2c_queue_t * q)
{
    q->queue_pos++;
    q->tx_send_idx=0;

    if ((q->queue_pos >= q->queue_end_pos) || (q->queue_pos >= (I2C_QUEUE_BUFFER_SIZE/q->queue_msg_size)) )
    {
        i2c_queue_clear(q);
    }
}

void lcd_send_queue_item(i2c_queue_t *q)
{
    uint8_t* tx_queue = &(q->b_queue[q->queue_pos * q->queue_msg_size]);
    q->tx_ongoing = 1;
    HAL_I2C_Master_Transmit_IT(&hi2c2,LCD_ADDR,tx_queue,q->queue_msg_size);
}

void lcd_OnDataTransmit(void)
{
    lcd_queue.tx_ongoing = 0;
    i2c_advance_queue(&lcd_queue);
}

void lcd_init(void)
{
    lcd_queue_init(&lcd_queue, 4);
    lcd_turnon_timer = Get10kTick();
}

void lcd_check_alive_rx(void)
{
    lcd_connected = 1;
    // return;
}

void lcd_check_alive(void)
{
    HAL_I2C_Master_Receive_IT(&hi2c2,LCD_ADDR,lcd_rx_buf,1);
    // HAL_I2C_Mem_Read_IT(&hi2c2,LCD_ADDR,0,1,lcd_rx_buf,1);
}

void lcd_queue_init_cmd(void)
{
    uint8_t data_u, data_l;
	uint8_t data_t[4];
    uint8_t cmd = lcd_init_commands[lcd_init_step];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0 -> bxxxx1100
	data_t[1] = data_u|0x08;  //en=0, rs=0 -> bxxxx1000
	data_t[2] = data_l|0x0C;  //en=1, rs=0 -> bxxxx1100
	data_t[3] = data_l|0x08;  //en=0, rs=0 -> bxxxx1000 
    lcd_queue_append(&lcd_queue,LCD_ADDR,data_t,4);

}

void lcd_mainloop(void)
{
    switch(lcd_state)
    {
        case LCD_STATE_OFF:
            if(lcd_connected)
            {
                lcd_state = LCD_STATE_INIT;
                lcd_init_step = 0;
                lcd_init_func_timer = Get10kTick();
            }
            if((Get10kTick()-lcd_turnon_timer) > 10000)
            {
            	lcd_check_alive();
            	lcd_turnon_timer = Get10kTick();
            }
        break;
        case LCD_STATE_INIT:
            if(Get10kTick()-lcd_init_func_timer > lcd_init_command_delays[lcd_init_step])
            {
                // lcd_queue_append(&lcd_queue,LCD_ADDR,&lcd_init_commands[lcd_init_step],1);
                lcd_queue_init_cmd();
                lcd_init_func_timer = Get10kTick();
                lcd_init_step++;
                if(lcd_init_step >= NUM_LCD_INIT_STEPS)
                {
                    lcd_state = LCD_STATE_IDLE;
                }
            }
        break;
        case LCD_STATE_IDLE:

        break;
        case LCD_STATE_TX:

        break;
    }


    if(!lcd_queue.tx_ongoing && (lcd_queue.queue_pos != lcd_queue.queue_end_pos) && (lcd_queue.queue_end_pos != 0))
    {
        lcd_send_queue_item(&lcd_queue);
    }
}
