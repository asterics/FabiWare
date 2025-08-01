/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PICO_SLEEP_H_
#define _PICO_SLEEP_H_

#ifdef RP2350   // low power support only available for RP2350

#include "pico.h"
#include "rosc.h"

#include "pico/aon_timer.h"

#ifdef __cplusplus
extern "C" {
#endif

/** \file sleep.h
 *  \defgroup hardware_sleep hardware_sleep
 *
 * Lower Power Sleep API
 *
 * The difference between sleep and dormant is that ALL clocks are stopped in dormant mode,
 * until the source (either xosc or rosc) is started again by an external event.
 * In sleep mode some clocks can be left running controlled by the SLEEP_EN registers in the clocks
 * block. For example you could keep clk_rtc running. Some destinations (proc0 and proc1 wakeup logic)
 * can't be stopped in sleep mode otherwise there wouldn't be enough logic to wake up again.
 *
 * \subsection sleep_example Example
 * \addtogroup hardware_sleep
 * \include hello_sleep.c

 */
typedef enum {
    DORMANT_SOURCE_NONE,
    DORMANT_SOURCE_XOSC,
    DORMANT_SOURCE_ROSC,
    DORMANT_SOURCE_LPOSC, // rp2350 only
} dormant_source_t;

/*! \brief Set all clock sources to the the dormant clock source to prepare for sleep.
 *  \ingroup hardware_sleep
 *
 * \param dormant_source The dormant clock source to use
 */
void sleep_run_from_dormant_source(dormant_source_t dormant_source);

/*! \brief Set the dormant clock source to be the crystal oscillator
 *  \ingroup hardware_sleep
 */
static inline void sleep_run_from_xosc(void) {
    sleep_run_from_dormant_source(DORMANT_SOURCE_XOSC);
}

#if !PICO_RP2040
static inline void sleep_run_from_lposc(void) {
    sleep_run_from_dormant_source(DORMANT_SOURCE_LPOSC);
}
#endif

/*! \brief Set the dormant clock source to be the ring oscillator
 *  \ingroup hardware_sleep
 */
static inline void sleep_run_from_rosc(void) {
    sleep_run_from_dormant_source(DORMANT_SOURCE_ROSC);
}

/*! \brief Send system to sleep until the specified time
 *  \ingroup hardware_sleep
 *
 * One of the sleep_run_* functions must be called prior to this call
 *
 * \param ts The time to wake up
 * \param callback Function to call on wakeup.
 */
void sleep_goto_sleep_until(struct timespec *ts, aon_timer_alarm_handler_t callback);

/*! \brief Send system to dormant until the specified time, note for RP2040 the RTC must be driven by an external clock
 *  \ingroup hardware_sleep
 *
 * One of the sleep_run_* functions must be called prior to this call
 *
 * \param ts The time to wake up
 * \param callback Function to call on wakeup.
 */
void sleep_goto_dormant_until(struct timespec *ts, aon_timer_alarm_handler_t callback);

/*! \brief Send system to sleep until the specified GPIO changes
 *  \ingroup hardware_sleep
 *
 * One of the sleep_run_* functions must be called prior to this call
 *
 * \param gpio_pin The pin to provide the wake up
 * \param edge true for leading edge, false for trailing edge
 * \param high true for active high, false for active low
 */

void sleep_goto_dormant_until_pin(uint gpio_pin, bool edge, bool high);

/*! \brief Send system to sleep until a leading high edge is detected on GPIO
 *  \ingroup hardware_sleep
 *
 * One of the sleep_run_* functions must be called prior to this call
 *
 * \param gpio_pin The pin to provide the wake up
 */
static inline void sleep_goto_dormant_until_edge_high(uint gpio_pin) {
    sleep_goto_dormant_until_pin(gpio_pin, true, true);
}

/*! \brief Send system to sleep until a high level is detected on GPIO
 *  \ingroup hardware_sleep
 *
 * One of the sleep_run_* functions must be called prior to this call
 *
 * \param gpio_pin The pin to provide the wake up
 */
static inline void sleep_goto_dormant_until_level_high(uint gpio_pin) {
    sleep_goto_dormant_until_pin(gpio_pin, false, true);
}

/*! \brief Reconfigure clocks to wake up properly from sleep/dormant mode
 *  \ingroup hardware_sleep
 *
 * This must be called immediately after continuing execution when waking up from sleep/dormant mode
 *
 */
void sleep_power_up(void);

#ifdef __cplusplus
}
#endif

#endif
#endif 