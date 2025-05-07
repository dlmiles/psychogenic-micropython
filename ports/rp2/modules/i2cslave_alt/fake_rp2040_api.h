/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2025 Darryl L. Miles
 *
 */

#ifndef __FAKE_RP2040_API_H
#define __FAKE_RP2040_API_H

#include <stdbool.h>
#include <stdint.h>
#include <pthread.h>

#include <stdio.h>    // printf
#include <string.h>   // memset/memcpy

#include "fake_rp2040_api_types.h"

#ifdef __cplusplus
extern "C" {
#endif


// specific handle for hardware block in silicon
extern struct i2c_inst i2c1_inst;

#define i2c1 (&i2c1_inst)


extern spin_lock_t SPIN_LOCKS[];

extern bool best_effort_wfe_or_timeout(absolute_time_t timeout_timestamp);

extern uint64_t time_us_64(void);

extern absolute_time_t get_absolute_time(void);

extern absolute_time_t make_timeout_time_us(uint64_t us);


extern uint next_striped_spin_lock_num(void);

extern int spin_lock_claim_unused(bool required);

extern spin_lock_t* spin_lock_init(uint lock_num);

extern uint32_t spin_lock_blocking(spin_lock_t* lock);

extern void spin_unlock(spin_lock_t* lock, uint32_t saved_irq);


extern void gpio_init(uint gpio);

extern void gpio_set_function(uint gpio, gpio_function_t fn);

extern void gpio_set_pulls(uint gpio, bool up, bool down);

extern void gpio_pull_up(uint gpio);


extern uint i2c_init(i2c_inst_t* i2c, uint baudrate);

extern void i2c_deinit(i2c_inst_t* i2c);


// original has this inlined
extern size_t i2c_get_read_available(i2c_inst_t* i2c);

// original has this inlined
extern size_t i2c_get_write_available(i2c_inst_t* i2c);


extern void i2c_slave_init(i2c_inst_t* i2c, uint8_t address, i2c_slave_handler_t handler);

extern void i2c_slave_deinit(i2c_inst_t* i2c);

extern void i2c_read_raw_blocking(i2c_inst_t* i2c, uint8_t* dst, size_t len);

extern uint8_t i2c_read_byte_raw(i2c_inst_t* i2c);

extern void i2c_write_raw_blocking(i2c_inst_t* i2c, const uint8_t* src, size_t len);

#ifdef __cplusplus
}
#endif


#ifdef __cplusplus
//////////////////////////////////////////////////////////////////////////////
///
/// Non RP2040 API used to mock the hardware (only available to C++ test code)
///

extern void fake_rp2040_reset(void);

extern int kick_i2c_receive_handler(size_t avail, size_t waitfor, uint8_t const* data = nullptr, size_t len = 0);
extern int kick_i2c_request_handler(size_t space, size_t waitfor);
extern int kick_i2c_finish_handler(void);
extern void kick_i2c_bogus_handler(int bogus_event);
#endif


#endif // __FAKE_RP2040_API_H
