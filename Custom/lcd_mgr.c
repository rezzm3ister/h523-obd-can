#include "lcd_mgr.h"
#include "can_mgr.h"
#include "gpio_mgr.h"

extern I2C_HandleTypeDef hi2c2;

i2c_queue_t lcd_queue;
uint8_t lcd_init_step = 0;
lcd_state_t lcd_state = LCD_STATE_OFF;
bool lcd_connected = 0;
static uint8_t lcd_rx_buf[8];

static uint8_t lcd_q1[8];
static uint8_t lcd_q2[8];
static uint8_t lcd_q3[8];
static uint8_t lcd_q4[8];
static uint8_t lcd_tmp_buf[64];
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

static uint32_t lcd_init_command_delays[NUM_LCD_INIT_STEPS] =
{
    600,
    600,
    20,
    200,
    200,
    60,
    60,
    200,
    60,
};

//timers
static uint32_t lcd_turnon_timer = 0;
static uint32_t lcd_init_func_timer = 0;
static uint32_t lcd_intermessage_timer = 0;
static uint32_t lcd_update_timer = 0;


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

bool lcd_send_queue_item(i2c_queue_t *q)
{
    if(q->queue_end_pos == 0)
    {
        return 0;
    }
    uint8_t* tx_queue = &(q->b_queue[q->queue_pos * q->queue_msg_size]);
    q->tx_ongoing = 1;
    HAL_I2C_Master_Transmit_IT(&hi2c2,LCD_ADDR,tx_queue,q->queue_msg_size);
    return 1;
}

void lcd_OnDataTransmit(void)
{
    lcd_intermessage_timer = Get10kTick();
    lcd_queue.tx_ongoing = 0;
    i2c_advance_queue(&lcd_queue);
    if(lcd_state == LCD_STATE_TX)
    {
        lcd_state = LCD_STATE_IDLE;
    }
    
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
    lcd_send_queue_item(&lcd_queue);
}

void lcd_queue_cmd(char data)
{
    uint8_t data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0 -> bxxxx1100
	data_t[1] = data_u|0x08;  //en=0, rs=0 -> bxxxx1000
	data_t[2] = data_l|0x0C;  //en=1, rs=0 -> bxxxx1100
	data_t[3] = data_l|0x08;  //en=0, rs=0 -> bxxxx1000 
    lcd_queue_append(&lcd_queue,LCD_ADDR,data_t,4);
}

void lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=0 -> bxxxx1101
	data_t[1] = data_u|0x09;  //en=0, rs=0 -> bxxxx1001
	data_t[2] = data_l|0x0D;  //en=1, rs=0 -> bxxxx1101
	data_t[3] = data_l|0x09;  //en=0, rs=0 -> bxxxx1001
	lcd_queue_append(&lcd_queue,LCD_ADDR,data_t,4);

	// HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 10);
	// HAL_I2C_Master_Transmit_IT (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4);
	// HAL_Delay(1);
}
//set cursor
void lcd_put_cur(int row, int col)
{
    switch (row)
    {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
    }

    lcd_queue_cmd (col);
}
void lcd_send_string (uint8_t* data, uint8_t len)
{
	// while (*str) lcd_send_data (*str++);
    for(uint8_t i = 0; i< len; i++)
    {
        lcd_send_data(data[i]);
    }
}

void lcd_mainloop(void)
{
    uint8_t temp_str[8] = "12345678";
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
                // lcd_send_queue_item(&lcd_queue);

                lcd_init_func_timer = Get10kTick();
                lcd_init_step++;
                if(lcd_init_step >= NUM_LCD_INIT_STEPS)
                {
                    lcd_state = LCD_STATE_IDLE;
                    lcd_intermessage_timer = Get10kTick();
                }
            }
        break;
        case LCD_STATE_IDLE:
            if((Get10kTick() - lcd_intermessage_timer > 10))
            {
                if(lcd_send_queue_item(&lcd_queue))
                {
                    lcd_state = LCD_STATE_TX;
                }
                
            }
        break;
        case LCD_STATE_TX:
            
        // case LCD_STATE_TX_WAIT:
        break;
        default:
        break;
    }
    if((Get10kTick() - lcd_update_timer > 1000) && lcd_state > LCD_STATE_INIT)
    {
        switch(gpio_GetLcdMode())
		{
			case LCD_MODE_TUNER:
				sprintf(lcd_q1,"AFR:");
				sprintf(lcd_tmp_buf,"%f",((float)can_GetActualAFR()) * can_GetAFRMultiplier());
				memcpy(&lcd_q1[4],lcd_tmp_buf,4);
				sprintf(lcd_q2,"TGT:");
				sprintf(lcd_tmp_buf,"%f",((float)can_GetActualAFRTGT()) * can_GetAFRTGTMultiplier());
				memcpy(&lcd_q2[4],lcd_tmp_buf,4);
				sprintf(lcd_q3,"IGN:");
				sprintf(lcd_tmp_buf,"%f",((float)can_GetActualIgnAdv()) * can_GetIgnAdvMultiplier());
				memcpy(&lcd_q3[4],lcd_tmp_buf,4);
				sprintf(lcd_q4,"IAT:");
				sprintf(lcd_tmp_buf,"%f",((float)can_GetActualIAT()) * can_GetIATMultiplier());
				memcpy(&lcd_q4[4],lcd_tmp_buf,4);
				break;
			case LCD_MODE_NORMAL: //specific to mazda for now
				sprintf(lcd_q1,"AFR:");
				sprintf(lcd_tmp_buf,"%f",((float)can_GetActualAFR()) * can_GetAFRMultiplier());
				memcpy(&lcd_q1[4],lcd_tmp_buf,4);
				sprintf(lcd_q2,"TFT:");
				sprintf(lcd_tmp_buf,"%f",((float)can_GetActualAtfTemp()) * can_GetAtfTempMultiplier());
				memcpy(&lcd_q2[4],lcd_tmp_buf,4);
				sprintf(lcd_q3,"CLT:");
				sprintf(lcd_tmp_buf,"%f",((float)can_GetActualCLT()) * can_GetCLTMultiplier());
				memcpy(&lcd_q3[4],lcd_tmp_buf,4);
				sprintf(lcd_q4,"EOT:");
				sprintf(lcd_tmp_buf,"%f",((float)can_GetActualOilTemp()) * can_GetOilTempMultiplier());
				memcpy(&lcd_q4[4],lcd_tmp_buf,4);

				break;
		}
        lcd_put_cur(0,0);
        lcd_send_string(lcd_q1,8);
        lcd_put_cur(0,8);
        lcd_send_string(lcd_q2,8);
        lcd_put_cur(1,0);
        lcd_send_string(lcd_q3,8);
        lcd_put_cur(1,8);
        lcd_send_string(lcd_q4,8);
        lcd_update_timer = Get10kTick();
    }
}
