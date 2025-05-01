/*
 * 
 * Copyright (c) 2025 Pat Deegan, https://psychogenic.com
 * Based on code from https://github.com/raspberrypi/pico-examples/tree/master/i2c/slave_mem_i2c
 * 
 * Copyright (c) 2021 Valentin Milea <valentin.milea@gmail.com>
 * Copyright (c) 2023 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <hardware/i2c.h>
#include <pico/i2c_slave.h>
#include <pico/stdlib.h>
#include <pico/sync.h>
#include <pico/multicore.h>
#include <stdio.h>
#include <string.h>

#include "./slave_mem_i2c.h"

//define USE_OUTDATA_LOCK


#define XFER_BUF_VOLATILITY     volatile

XFER_BUF_VOLATILITY xfer_buffer in_context;
XFER_BUF_VOLATILITY xfer_buffer out_context;
uint8_t in_buffer[I2CSLAVE_MEMBUF_LEN];
bool is_receiving = 0;


static data_in_callback callback_datain = NULL;
static data_out_done_callback callback_dataout_done = NULL;

volatile uint8_t busy_reading_or_writing = 0;


#ifdef USE_OUTDATA_LOCK
spin_lock_t *outdata_lock = NULL;

static void init_outdata_lock() {

    if (outdata_lock == NULL) {
        outdata_lock = spin_lock_init(next_striped_spin_lock_num());


    }

}
#endif

// Our handler is called from the I2C ISR, so it must complete quickly. Blocking calls /
// printing to stdio may interfere with interrupt handling.
static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    #ifdef USE_OUTDATA_LOCK
    uint32_t save = 0;
    #endif
    switch (event) {
    case I2C_SLAVE_RECEIVE: // master has written some data
        // save into memory
        busy_reading_or_writing = 1;
        is_receiving = 1;
        #ifdef USE_OUTDATA_LOCK
        save = spin_lock_blocking(outdata_lock);
        #endif
        if (in_context.mem_index >= I2CSLAVE_MEMBUF_LEN) {
            in_context.mem_index = 0;
        }
        in_context.mem[in_context.mem_index] = i2c_read_byte_raw(i2c);
        in_context.mem_index++;
        in_context.mem_len = in_context.mem_index;
        #ifdef USE_OUTDATA_LOCK
        spin_unlock(outdata_lock, save);
        #endif
        break;
    case I2C_SLAVE_REQUEST: // master is requesting data
        is_receiving = 0;
        busy_reading_or_writing = 1;
        #ifdef USE_OUTDATA_LOCK
        save = spin_lock_blocking(outdata_lock);
        #endif
        if (out_context.mem_index >= I2CSLAVE_MEMBUF_LEN) {
            out_context.mem_index = 0;
            out_context.mem_len = 0;
        }
        
        if (out_context.mem_index >= out_context.mem_len) {
            i2c_write_byte_raw(i2c, 0xff);
        } else {
            i2c_write_byte_raw(i2c, out_context.mem[out_context.mem_index]);
        }
        
        out_context.mem_index++;
        #ifdef USE_OUTDATA_LOCK
        spin_unlock(outdata_lock, save);
        #endif
        break;
    case I2C_SLAVE_FINISH: // master has signalled Stop / Restart
        busy_reading_or_writing = 0;
        if (is_receiving) {
            if (in_context.mem_len) {
                // have data in, notify userspace
                
                #ifdef USE_OUTDATA_LOCK
                save = spin_lock_blocking(outdata_lock);
                #endif 
                
                uint8_t len = in_context.mem_len;
                in_context.mem_index = 0;
                in_context.mem_len = 0;
                memcpy(in_buffer, (const void*)in_context.mem, len );
                memset((void*)in_context.mem, 0, len);
                #ifdef USE_OUTDATA_LOCK
                spin_unlock(outdata_lock, save);
                #endif
                if (callback_datain != NULL) {
                    callback_datain(len, in_buffer);
                }
            }
        } else {
            #ifdef USE_OUTDATA_LOCK
            save = spin_lock_blocking(outdata_lock);
            #endif
            
            if (out_context.mem_index >= out_context.mem_len) {
                // notify write done
                if (callback_dataout_done != NULL) {
                    callback_dataout_done();
                }
            }
            #ifdef USE_OUTDATA_LOCK
            spin_unlock(outdata_lock, save);
            #endif
            
        }
        
        break;
    default:
        break;
    }
}



void slvmem_set_data_out(uint8_t len, uint8_t * bts) {
    #ifdef USE_OUTDATA_LOCK
    init_outdata_lock();
    uint32_t save = spin_lock_blocking(outdata_lock);
    #endif
    memcpy((void*)out_context.mem, bts, len);
    out_context.mem_len = len;
    out_context.mem_index = 0;
    #ifdef USE_OUTDATA_LOCK
    spin_unlock(outdata_lock, save);
    #endif

    
}

void slvmem_flush_output(void) {
    
    #ifdef USE_OUTDATA_LOCK
    init_outdata_lock();
    uint32_t save = spin_lock_blocking(outdata_lock);
    #endif 
    
    out_context.mem_len = 0;
    out_context.mem_index = 0;
    #ifdef USE_OUTDATA_LOCK
    spin_unlock(outdata_lock, save);
    #endif
}
void slvmem_i2c_deinit(void) {
    i2c_deinit(I2CSLAVE_DEVICE);
}

volatile uint8_t slvmem_is_busy(void)
{
    return busy_reading_or_writing;
    
}
void slvmem_set_callbacks(
    data_in_callback cb_datain,
    data_out_done_callback cb_dataout_done)
{
    
    callback_datain = cb_datain;
    callback_dataout_done = cb_dataout_done;
}

void slvmem_i2c_init(uint8_t sda_pin, uint8_t scl_pin, uint8_t address, 
    uint baudrate,
    data_in_callback cb_datain,
    data_out_done_callback cb_dataout_done,
    uint8_t use_pullups) {
    
    slvmem_set_callbacks(cb_datain, cb_dataout_done); 
    
    gpio_init(sda_pin);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    if (use_pullups) {
        gpio_pull_up(sda_pin);
    }

    gpio_init(scl_pin);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    
    if (use_pullups) {
        gpio_pull_up(scl_pin);
    }

    i2c_init(I2CSLAVE_DEVICE, baudrate);
    // configure I2C0 for slave mode
    i2c_slave_init(I2CSLAVE_DEVICE, address, &i2c_slave_handler);
}


