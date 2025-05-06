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
 * 2) optional, either use polling (recommended) or callbacks 
 *    (still get corrupted somehow, sometimes). If using callbacks
 *    set those up for data in and tx done using 
 *    set_datatxdone_callback and
 *    set_datain_callback
 * 
 * 3) call initialize() to actually start processing
 * 
 * 4) call write_bytes(LEN, BYTES) to set the output buffer
 *    for reads from master
 * 
 * 5) if polling, check tx_done and have_pending_data and then
 *    write_bytes or pending_data_into() as appropriate
 * 
 * Example
 * 
 * 
 *  import i2cslave
 * 
 *  def set_outdata(bts):
 *      return i2cslave.write_bytes(len(bts), bts)
 *      
 *  def start_i2c():
 *      sclpin = 3
 *      sdapin = 2
 *      outbytes = bytearray('hello!', 'ascii')
 *      i2cslave.setup(SlaveAddy, sclpin, sdapin)
 *      # i2cslave.set_datain_callback(rcv_data_cb)
 *      # i2cslave.set_datatxdone_callback(tx_done_cb)
 *      return i2cslave.initialize()
 * 
 *  
 * 
 * start_i2c()
 * set_outdata(b'this will go out when I am read')
 * while True:
 *    in_bytes = bytearray(256)
 *    if i2cslave.have_pending_data():
 *         numbytes = i2cslave.pending_data_into(in_bytes)
 *         # do something with
 *    if i2cslave.tx_done():
 *         send_more_data()
 * 
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
typedef struct _callback_store_t {
    mp_obj_t datain;
    mp_obj_t datatxdone;
} callback_store_t;

// Static storage instance
static callback_store_t * callback_store = NULL;


// current state bundle
typedef struct _i2c_tx_state_t {
        uint8_t have_pending_in;
        uint8_t i2c_tx_done;
        uint8_t init_done;
} i2c_tx_state_t;

static i2c_tx_state_t i2c_state = {0};


/* settings for I2C device */
#define I2CSLAVE_PULLUPS_DEFAULT    1
typedef struct _i2c_settings {
    uint8_t address;
    uint8_t pin_sda;
    uint8_t pin_scl;
    uint    baudrate;
    uint8_t use_pullups;
} i2c_settings_t;

static i2c_settings_t i2c_settings = {0};


static void i2cslave_init_cb_storage(void) {
    
    I2CS_DEBUG("CB STORAGE INIT\n");
    if (callback_store == NULL) {
        
        I2CS_DEBUG("... first time!\n");
        callback_store = m_new(callback_store_t, 1); // Allocate GC-tracked memory
        callback_store->datain = MP_OBJ_NULL; // Initialize to null
        callback_store->datatxdone = MP_OBJ_NULL; // Initialize to null
    }
}


// setup function to specify params
// .setup(address, scl, sda, baud, [use_pullups])
static mp_obj_t i2cslave_setup(mp_uint_t n_args, const mp_obj_t *args) {
    
    mp_int_t address = mp_obj_get_int(args[0]);
    mp_int_t scl_pin = mp_obj_get_int(args[1]);
    mp_int_t sda_pin = mp_obj_get_int(args[2]);
    mp_int_t baudrate = mp_obj_get_int(args[3]);
    
    mp_int_t pullups = I2CSLAVE_PULLUPS_DEFAULT;
    if (n_args >= 5) {
        pullups = mp_obj_get_int(args[4]);
    }
    
    // Validate parameters
    if (address < 0 || address > 127) {
        mp_raise_ValueError(MP_ERROR_TEXT("address must be 0-127"));
    }
    i2c_settings.address = (uint8_t)address;
    if (scl_pin < 0 || scl_pin > 29 || sda_pin < 0 || sda_pin > 29) {
        mp_raise_ValueError(MP_ERROR_TEXT("pins must be 0-29"));
    }
    if (scl_pin == sda_pin) {
        mp_raise_ValueError(MP_ERROR_TEXT("SCL and SDA pins must be different"));
    }
    i2c_settings.pin_sda = (uint8_t)sda_pin;
    i2c_settings.pin_scl = (uint8_t)scl_pin;
    
    i2c_settings.baudrate = baudrate;
    i2c_settings.use_pullups = (uint8_t)pullups;
    
    I2CS_DEBUG("i2c setup done!\n");
    
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(i2cslave_setup_obj, 4, 5, i2cslave_setup);





// Set data received callback function
static mp_obj_t i2cslave_set_datain_callback(mp_obj_t callback_obj) {
    if (callback_obj != mp_const_none && !mp_obj_is_callable(callback_obj)) {
        mp_raise_TypeError(MP_ERROR_TEXT("callback must be callable or None"));
    }
    I2CS_DEBUG("Setting up datain cb: ");
    I2CS_DEBUGOBJ(callback_obj);
    i2cslave_init_cb_storage();
    callback_store->datain = callback_obj;
    I2CS_DEBUGOBJ(callback_store->datain);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(i2cslave_set_datain_callback_obj, i2cslave_set_datain_callback);



// Set all data transmitted callback function
// .set_datatx_done_callback(somefunction_to_call)
static mp_obj_t i2cslave_set_datatx_done_callback(mp_obj_t callback_obj) {
    if (callback_obj != mp_const_none && !mp_obj_is_callable(callback_obj)) {
        mp_raise_TypeError(MP_ERROR_TEXT("callback must be callable or None"));
    }
    
    I2CS_DEBUG("Setting up data tx done cb: ");
    I2CS_DEBUGOBJ(callback_obj);
    i2cslave_init_cb_storage();
    callback_store->datatxdone = callback_obj;
    I2CS_DEBUGOBJ(callback_store->datatxdone);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(i2cslave_set_datatx_done_callback_obj, i2cslave_set_datatx_done_callback);




static uint8_t cbsched_numbytes = 0;
static uint8_t cbsched_contents[I2CSLAVE_MEMBUF_LEN];

// function used from i2c handler side to trigger callback on rcv, if set
static void i2cslave_trigger_datain_callback(uint8_t numbytes, uint8_t *bts) {
    
    i2c_state.have_pending_in = 1; // flag it, for polling
    
    /* always copy, so this works with cb and polling */
    cbsched_numbytes = numbytes;
    memcpy(cbsched_contents, bts, numbytes);
    
    
    if (callback_store != NULL && callback_store->datain != MP_OBJ_NULL && callback_store->datain != mp_const_none) {
        // I2CS_DEBUG("schdi: ");
        // I2CS_DEBUGOBJ(callback_store->datain);
        mp_sched_schedule(callback_store->datain, mp_const_none);
        // I2CS_DEBUG("done\n");
    }
}


// function used from i2c handler side to trigger callback on 
// out buffer all transmitted
static void i2cslave_data_out_done_callback() {
    
    i2c_state.i2c_tx_done = 1; // flag it, for polling
    
    if (callback_store != NULL && callback_store->datatxdone != MP_OBJ_NULL && callback_store->datatxdone != mp_const_none) {
        
        // I2CS_DEBUG("schdo: ");
        // I2CS_DEBUGOBJ(callback_store->datatxdone);
        mp_sched_schedule(callback_store->datatxdone, mp_const_none);
        // I2CS_DEBUG("done\n");
    }
}



// Initialize function
// initialize() -- call when ready, after setup() is done
static mp_obj_t i2cslave_initialize(void) {
    
    
    if (i2c_settings.address == 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("call setup() first"));
    }
    
    
    
    if (i2c_state.init_done) {
        // be sure we (re)set the callbacks regardless
        slvmem_set_callbacks(i2cslave_trigger_datain_callback, i2cslave_data_out_done_callback);
        
        mp_raise_ValueError(MP_ERROR_TEXT("init was already done"));
    }
    
    i2c_state.init_done = 1;
    
    slvmem_i2c_init(i2c_settings.pin_sda, i2c_settings.pin_scl, i2c_settings.address, 
        i2c_settings.baudrate,
        i2cslave_trigger_datain_callback,
        i2cslave_data_out_done_callback,
        i2c_settings.use_pullups);
        
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(i2cslave_initialize_obj, i2cslave_initialize);










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
    
    i2c_state.have_pending_in = 0; // clear the flag, data returned
    
    mp_obj_t sz = mp_obj_new_int(size);
    return sz;
}
static MP_DEFINE_CONST_FUN_OBJ_1(i2cslave_pending_data_into_obj, i2cslave_pending_data_into);

static mp_obj_t i2cslave_have_pending_data() {
    
    mp_obj_t v =  mp_obj_new_int(i2c_state.have_pending_in);
    // I2CS_DEBUGOBJ(v);
    return v;
}
static MP_DEFINE_CONST_FUN_OBJ_0(i2cslave_have_pending_data_obj, i2cslave_have_pending_data);


static mp_obj_t i2cslave_tx_done() {
    
    mp_obj_t rv = mp_obj_new_int(i2c_state.i2c_tx_done);
    i2c_state.i2c_tx_done = 0; // question has been asked, clear that flag
    return rv;
}
static MP_DEFINE_CONST_FUN_OBJ_0(i2cslave_tx_done_obj, i2cslave_tx_done);









// Initialize function
// flush_output() -- call to cancel any pending output
static mp_obj_t i2cslave_flush_output(void) {
    while (slvmem_is_busy()) {
        ;
    }
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
    if (i2c_state.init_done || force_deinit) {
        i2c_state.init_done = 0;
        slvmem_i2c_deinit();
    }
        
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(i2cslave_deinitialize_obj, 0, 1, i2cslave_deinitialize);




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
    while (slvmem_is_busy()) {
        ;
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
    { MP_ROM_QSTR(MP_QSTR_have_pending_data), MP_ROM_PTR(&i2cslave_have_pending_data_obj) },
    { MP_ROM_QSTR(MP_QSTR_tx_done), MP_ROM_PTR(&i2cslave_tx_done_obj) },
    
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