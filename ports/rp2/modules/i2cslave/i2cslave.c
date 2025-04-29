/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2025 Pat Deegan, https://psychogenic.com
 *
 * 
 * Usage from python:
 * 
 * 1) call setup(ADDRESS, SCL, SDA, BAUDRATE) to specify params
 * 
 * 2) optional, but probably needed, create a callback for 
 *    received data  CB(LEN, BYTES)
 * 
 * 3) optional, create a callback to know when all queued 
 *    data has been read by master, CB()
 * 
 * 4) call initialize() to actually start processing
 * 
 * 5) call write_bytes(LEN, BYTES) to set the output buffer
 *    for reads from master
 * 
 * Example
 * 
 * 
 *  import i2cslave
 * 
 *  def rcv_data_cb(numbytes, bts):
 *      print(f"GOT {numbytes} DATA: {bts}")
 *      
 *  def tx_done_cb():
 *      print("Data transmitted!")
 *      
 *  def set_outdata(bts):
 *      return i2cslave.write_bytes(len(bts), bts)
 *      
 *  def start_i2c():
 *      sclpin = 3
 *      sdapin = 2
 *      outbytes = bytearray('hello!', 'ascii')
 *      i2cslave.setup(SlaveAddy, sclpin, sdapin)
 *      i2cslave.set_datain_callback(rcv_data_cb)
 *      i2cslave.set_datatxdone_callback(tx_done_cb)
 *      return i2cslave.initialize()
 * 
 * start_i2c()
 * set_outdata(b'this will go out when I am read')
 * 
 * 
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
 

#include "py/mpconfig.h"
#include "py/qstr.h"
#include "py/misc.h"      
#include "py/obj.h"      
#include "py/runtime.h"   
#include "py/objstr.h"   
#include "slave_mem_i2c.h"
#include <string.h>

/* callbacks to uPython space */
static mp_obj_t i2cslave_datain_callback = MP_OBJ_NULL;
static mp_obj_t i2cslave_datatxdone_callback = MP_OBJ_NULL;

/* settings for I2C device */
static uint8_t i2c_address = 0;
static uint8_t i2c_pin_sda = 0;
static uint8_t i2c_pin_scl = 0;
static uint    i2c_baudrate = 0;
static uint8_t i2c_use_pullups = 1;

/* flag to know if already done */
static uint8_t i2c_init_done = 0;



// Set data received callback function
static mp_obj_t i2cslave_set_datain_callback(mp_obj_t callback_obj) {
    if (callback_obj != mp_const_none && !mp_obj_is_callable(callback_obj)) {
        mp_raise_TypeError(MP_ERROR_TEXT("callback must be callable or None"));
    }
    i2cslave_datain_callback = callback_obj;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(i2cslave_set_datain_callback_obj, i2cslave_set_datain_callback);



// Set all data transmitted callback function
// .set_datatx_done_callback(somefunction_to_call)
static mp_obj_t i2cslave_set_datatx_done_callback(mp_obj_t callback_obj) {
    if (callback_obj != mp_const_none && !mp_obj_is_callable(callback_obj)) {
        mp_raise_TypeError(MP_ERROR_TEXT("callback must be callable or None"));
    }
    i2cslave_datatxdone_callback = callback_obj;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(i2cslave_set_datatx_done_callback_obj, i2cslave_set_datatx_done_callback);



static uint8_t cbsched_numbytes = 0;
static uint8_t cbsched_contents[I2CSLAVE_MEMBUF_LEN];

// function used from i2c handler side to trigger callback on rcv, if set
static void i2cslave_trigger_datain_callback(uint8_t numbytes, uint8_t *bts) {
    
    if (i2cslave_datain_callback != MP_OBJ_NULL && i2cslave_datain_callback != mp_const_none) {
        cbsched_numbytes = numbytes;
        memcpy(cbsched_contents, bts, numbytes);
        // mp_obj_t sz = mp_obj_new_int(numbytes);
        mp_sched_schedule(i2cslave_datain_callback, mp_const_none);
    }
}

static mp_obj_t i2cslave_pending_data_into(mp_obj_t ba_obj) {
    
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(ba_obj, &bufinfo, MP_BUFFER_WRITE);
    
    size_t size = bufinfo.len;
    if (size > cbsched_numbytes) {
        size = cbsched_numbytes;
    }
    if (size) {
        memcpy(bufinfo.buf, cbsched_contents, size);
    }
    mp_obj_t sz = mp_obj_new_int(size);
    return sz;
}
static MP_DEFINE_CONST_FUN_OBJ_1(i2cslave_pending_data_into_obj, i2cslave_pending_data_into);



// function used from i2c handler side to trigger callback on 
// out buffer all transmitted
static void i2cslave_data_out_done_callback() {
    if (i2cslave_datatxdone_callback != MP_OBJ_NULL && i2cslave_datatxdone_callback != mp_const_none) {
        mp_sched_schedule(i2cslave_datatxdone_callback, mp_const_none);
    }
}







// Initialize function
// initialize() -- call when ready, after setup() is done
static mp_obj_t i2cslave_initialize(void) {
    
    if (i2c_address == 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("call setup() first"));
    }
    
    
    
    if (i2c_init_done) {
        slvmem_set_callbacks(i2cslave_trigger_datain_callback, i2cslave_data_out_done_callback);
        
        mp_raise_ValueError(MP_ERROR_TEXT("init was already done"));
    }
    
    i2c_init_done = 1;
    
    slvmem_i2c_init(i2c_pin_sda, i2c_pin_scl, i2c_address, 
        i2c_baudrate,
        i2cslave_trigger_datain_callback,
        i2cslave_data_out_done_callback,
        i2c_use_pullups);
        
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(i2cslave_initialize_obj, i2cslave_initialize);



// Initialize function
// flush_output() -- call to cancel any pending output
static mp_obj_t i2cslave_flush_output(void) {
    
    slvmem_flush_output();
        
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(i2cslave_flush_output_obj, i2cslave_flush_output);




// Deinit function
// .deinitialize([FORCE])
static mp_obj_t i2cslave_deinitialize(mp_uint_t n_args, const mp_obj_t *args) {
    bool force_deinit = 0;
    
    if (n_args) {
        force_deinit = mp_obj_is_true(args[0]);
    } 
    if (i2c_init_done || force_deinit) {
        i2c_init_done = 0;
        slvmem_i2c_deinit();
    }
        
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(i2cslave_deinitialize_obj, 0, 1, i2cslave_deinitialize);


// setup function to specify params
// .setup(address, scl, sda, baud, [use_pullups])
static mp_obj_t i2cslave_setup(mp_uint_t n_args, const mp_obj_t *args) {
    mp_int_t address = mp_obj_get_int(args[0]);
    mp_int_t scl_pin = mp_obj_get_int(args[1]);
    mp_int_t sda_pin = mp_obj_get_int(args[2]);
    mp_int_t baudrate = mp_obj_get_int(args[3]);
    
    mp_int_t pullups = i2c_use_pullups;
    if (n_args >= 5) {
        pullups = mp_obj_get_int(args[4]);
    }
    // Validate parameters
    if (address < 0 || address > 127) {
        mp_raise_ValueError(MP_ERROR_TEXT("address must be 0-127"));
    }
    i2c_address = (uint8_t)address;
    if (scl_pin < 0 || scl_pin > 29 || sda_pin < 0 || sda_pin > 29) {
        mp_raise_ValueError(MP_ERROR_TEXT("pins must be 0-29"));
    }
    if (scl_pin == sda_pin) {
        mp_raise_ValueError(MP_ERROR_TEXT("SCL and SDA pins must be different"));
    }
    i2c_pin_sda = (uint8_t)sda_pin;
    i2c_pin_scl = (uint8_t)scl_pin;
    
    i2c_baudrate = baudrate;
    i2c_use_pullups = (uint8_t)pullups;
    
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(i2cslave_setup_obj, 4, 5, i2cslave_setup);







// Write bytes: queue data out to return on reads from master
// .write(len, bytes)
static mp_obj_t i2cslave_write_bytes(mp_uint_t n_args, const mp_obj_t *args) {
    // Extract bytelen (integer)
    int bytelen = mp_obj_get_int(args[0]);
    // Extract bts (byte array)
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[1], &bufinfo, MP_BUFFER_READ);
    uint8_t *bts = (uint8_t *)bufinfo.buf;
    if (bufinfo.len < bytelen) {
        mp_raise_ValueError(MP_ERROR_TEXT("buffer too small"));
    }
    
    slvmem_set_data_out(bytelen, bts);
    int result = bytelen;
    return mp_obj_new_int(result);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(i2cslave_write_bytes_obj, 2, 2, i2cslave_write_bytes);


// Define the module’s attribute table
static const mp_rom_map_elem_t i2cslave_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_i2cslave) },
    { MP_ROM_QSTR(MP_QSTR_setup), MP_ROM_PTR(&i2cslave_setup_obj) },
    { MP_ROM_QSTR(MP_QSTR_initialize), MP_ROM_PTR(&i2cslave_initialize_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&i2cslave_deinitialize_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_datain_callback), MP_ROM_PTR(&i2cslave_set_datain_callback_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_datatxdone_callback), MP_ROM_PTR(&i2cslave_set_datatx_done_callback_obj) },
    { MP_ROM_QSTR(MP_QSTR_write_bytes), MP_ROM_PTR(&i2cslave_write_bytes_obj)},
    { MP_ROM_QSTR(MP_QSTR_pending_data_into), MP_ROM_PTR(&i2cslave_pending_data_into_obj)},
    { MP_ROM_QSTR(MP_QSTR_flush_output), MP_ROM_PTR(&i2cslave_flush_output_obj)}
};
static MP_DEFINE_CONST_DICT(i2cslave_module_globals, i2cslave_module_globals_table);

// Define the module object
const mp_obj_module_t i2cslave_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&i2cslave_module_globals,
};

// Register the module
MP_REGISTER_MODULE(MP_QSTR_i2cslave, i2cslave_module);