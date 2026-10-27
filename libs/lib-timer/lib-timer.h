#ifndef LIB_TIMER_H
#define LIB_TIMER_H

#include "custom_main.h"
#include "includes.h"

// Time conversion macros (10 ticks per millisecond)
#define MS_TO_TICKS(ms) ((ms) * 10)
#define SEC_TO_TICKS(s) ((s) * 10000)

// Define the status enum
typedef enum {
    HPTIM_OFF,
    HPTIM_ON,
    HPTIM_EXPIRED
} HPTIM_STATUS;

// Define the timer structure
typedef struct {
    uint32_t set_time;
    uint32_t end_time;
    HPTIM_STATUS status;
} HPTim_t;

// Initialization macro
#define HPTIM_INIT(timer_ptr) \
    (timer_ptr)->set_time = 0; \
    (timer_ptr)->end_time = 0; \
    (timer_ptr)->status = HPTIM_OFF

// Function prototypes

void hptim_start(HPTim_t *timer, uint32_t duration);
void hptim_update(HPTim_t *timer);
void hptim_stop(HPTim_t *timer);
HPTIM_STATUS hptim_get_status(const HPTim_t *timer);

#endif // LIB_TIMER_H
