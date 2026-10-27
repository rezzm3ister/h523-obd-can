#include "lib-timer.h"
#include "tim.h" // Required for Get10kTick()

/**
 * @brief Starts a timer.
 * 
 * Sets status to ON, sets the current time to Get10kTick(), and sets the expiry time.
 * 
 * @param timer Pointer to timer struct.
 * @param duration Duration in 10kticks.
 */
void hptim_start(HPTim_t *timer, uint32_t duration) {
    uint32_t current_time = Get10kTick();
    timer->set_time = current_time;
    timer->end_time = current_time + duration;
    timer->status = HPTIM_ON;
}

/**
 * @brief Updates the timer status.
 * 
 * Checks current time against end time, updates set time, and handles expiration.
 * 
 * @param timer Pointer to timer struct.
 */
void hptim_update(HPTim_t *timer) {
    uint32_t current_time = Get10kTick();

    if (timer->status == HPTIM_ON) {
        if (current_time >= timer->end_time) {
            // Expired
            timer->set_time = 0;
            timer->end_time = 0;
            timer->status = HPTIM_EXPIRED;
        } else {
            // Still running - writing new value to set_time as per requirement
            timer->set_time = current_time; 
        }
    }
}

/**
 * @brief Stops the timer.
 * 
 * Sets start and end to 0 and status to EXPIRED.
 * 
 * @param timer Pointer to timer struct.
 */
void hptim_stop(HPTim_t *timer) {
    timer->set_time = 0;
    timer->end_time = 0;
    timer->status = HPTIM_EXPIRED; 
}

/**
 * @brief Returns the status of the timer.
 * 
 * @param timer Pointer to timer struct.
 * @return HPTIM_STATUS
 */
HPTIM_STATUS hptim_get_status(const HPTim_t *timer) {
    return timer->status;
}
