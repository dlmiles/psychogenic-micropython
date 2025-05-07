/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2025 Pat Deegan, https://psychogenic.com
 * Copyright (c) 2025 Darryl L. Miles
 *
 *
 * Usage from python:
 *
 * 1) call setup(ADDRESS, SCL, SDA, BAUDRATE) to specify params
 *
 * 2) call initialize() and check return True to finalize hardware
 *
 * 3) call tx_write_bytes(BYTES, OFF, LEN) to set the TX data to send
 *    to I2C MASTER.
 *
 * 4) optionally call xfer_buffer_rx_available() to see if there is
 *    pending data received from I2C MASTER.
 *
 * 5) optionally call rx_peek_bytes_info(BA, 0, LEN) to peek at data from
 *    the queue.
 *
 * 6) call rx_read_bytes_info(BA, LEN) to read the next block of 8-byte
 *    data out of the queue.
 *
 * The i2cslave API is a Static Singleton pattern with a thread-safe Python
 *    API that does not require any synchronouzation.
 *
 * Example
 *
 *
 *  import i2cslave
 *
 *  def set_outdata(data):
 *      return i2cslave.tx_write_bytes(data)
 *
 *  def start_i2c():
 *      i2c_address = 0x51
 *      scl_pin = 3
 *      sda_pin = 2
 *      i2cslave.setup(i2c_address, scl_pin, sda_pin)
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
#include "lib/i2cslave_sock_seqpacket.h"
#include <string.h>


/* settings for I2C device */
static uint8_t i2c_address = 0;
static uint8_t i2c_pin_sda = 0;
static uint8_t i2c_pin_scl = 0;
static uint    i2c_baudrate = 0;
static uint8_t i2c_use_pullups = 1;

static inline bool
mp_obj_is_alive(const mp_obj_t* o)
{
    // FIXME maybe only half of this is needed, is None a type, what type is it, is that type an object ?
    return mp_obj_is_obj(o) && o != mp_const_none; // return o and o is not None
}


// .rx_peek_bytes_info(dest, offpeek, len, [minpeek:int=0])
static mp_obj_t
i2cslave_alt_rx_peek_bytes(mp_uint_t n_args, const mp_obj_t* args)
{
    bool can_copy = mp_obj_is_alive(args[0]);
    mp_buffer_info_t bufinfo;
    if(can_copy)
        mp_get_buffer_raise(args[0], &bufinfo, MP_BUFFER_WRITE);
    else
        bufinfo.len = 0; // because we use it below

    size_t offpeek = (n_args > 1) ? mp_obj_get_int(args[1]) : 0;
    size_t len     = (n_args > 2) ? mp_obj_get_int(args[2]) : bufinfo.len;
    size_t minpeek = (n_args > 3) ? mp_obj_get_int(args[3]) : 0;
    // offpeek relates to peek-ahead-offset not copy-into-offset so no range check here
    if(offpeek < 0 || len < 0 || minpeek < 0)
        mp_raise_ValueError(MP_ERROR_TEXT("invalid arguments"));
    if(can_copy && bufinfo.len < len)
        mp_raise_ValueError(MP_ERROR_TEXT("invalid arguments"));

    uint8_t* buf  = (uint8_t*) bufinfo.buf; // cast to byte*
    uint8_t* dest = (can_copy) ? &buf[0] : NULL; // always copy into offset=0
    int plen = slvmem_rx_peek_bytes(dest, offpeek, len, minpeek);
    return mp_obj_new_int(plen);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(i2cslave_alt_rx_peek_bytes_obj, 1, 4, i2cslave_alt_rx_peek_bytes);


// .rx_read_bytes_info(dest, offset, len, [minread:int=0])
static mp_obj_t
i2cslave_alt_rx_read_bytes(mp_uint_t n_args, const mp_obj_t* args)
{
    bool can_copy = mp_obj_is_alive(args[0]);
    mp_buffer_info_t bufinfo;
    if(can_copy)
        mp_get_buffer_raise(args[0], &bufinfo, MP_BUFFER_WRITE);
    else
        bufinfo.len = 0; // because we use it below

    size_t offset  = (n_args > 1) ? mp_obj_get_int(args[1]) : 0;
    size_t len     = (n_args > 2) ? mp_obj_get_int(args[2]) : bufinfo.len;
    size_t minread = (n_args > 3) ? mp_obj_get_int(args[3]) : 0;
    if(offset < 0 || len < 0 || minread < 0)
        mp_raise_ValueError(MP_ERROR_TEXT("invalid arguments"));
    // offset only needs bounds check when copy will occur
    if(can_copy && (bufinfo.len < (offset+len) || bufinfo.len < offset || bufinfo.len < len))
        mp_raise_ValueError(MP_ERROR_TEXT("invalid arguments"));

    uint8_t* buf  = (uint8_t*) bufinfo.buf; // cast to byte*
    uint8_t* dest = (can_copy) ? &buf[offset] : NULL; // apply offset
    int rlen = slvmem_rx_read_bytes(dest, len, minread);
    return mp_obj_new_int(rlen);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(i2cslave_alt_rx_read_bytes_obj, 1, 4, i2cslave_alt_rx_read_bytes);


// Initialize function
// .initialize(): int -- call when ready, after setup() is done
static mp_obj_t
i2cslave_alt_initialize(mp_uint_t n_args, const mp_obj_t* args)
{
    uint32_t init_mode = (n_args > 0) ? mp_obj_get_int(args[0]) : INIT_M_DEFAULT;
#if 0
    // this is range 1..127 checked in slvmem_i2c_initialize() and we return False on error to caller
    if(i2c_address == 0)
        mp_raise_ValueError(MP_ERROR_TEXT("call setup() first"));
#endif
    const i2c_slave_config_t cfg_values = {
        .sda_pin     = i2c_pin_sda,
        .scl_pin     = i2c_pin_scl,
        .address     = i2c_address,
        .baudrate    = i2c_baudrate,
        .use_pullups = i2c_use_pullups,
        .nack_mode   = NACK_M_NONE
    };

    int res = slvmem_i2c_initialize(&cfg_values, init_mode);

    // special error value to raise exception as the low level driver targets C API
    if(res == INT_MIN)
        mp_raise_ValueError(MP_ERROR_TEXT("init was already done"));

    return mp_obj_new_int(res); // baud rate set
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(i2cslave_alt_initialize_obj, 0, 1, i2cslave_alt_initialize);


// Discard TX data state
// .tx_discard_all() -- call to cancel any pending output
static mp_obj_t
i2cslave_alt_tx_discard_all(void)
{
    slvmem_tx_discard_all();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(i2cslave_alt_tx_discard_all_obj, i2cslave_alt_tx_discard_all);


// Wait for TX flush to complete function (aka drain)
// .tx_drain_timeout_abs([timeout_abs: int]) -- call to gracefully try to allow tx drain to occur
static mp_obj_t
i2cslave_alt_tx_drain_timeout_abs(mp_uint_t n_args, const mp_obj_t* args)
{
    // both notify_abs and timeout_abs are 64bit integers
    uint64_t timeout_abs = (n_args > 0 && mp_obj_is_integer(args[0])) ? mp_obj_int_get_checked(args[0]) : 0;
    int res = slvmem_tx_drain_timeout_abs(timeout_abs);
    return mp_obj_new_int(res);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(i2cslave_alt_tx_drain_timeout_abs_obj, 0, 1, i2cslave_alt_tx_drain_timeout_abs);


// Wait for TX flush to complete function (aka drain)
// .tx_drain_timeout_us([timeout_us: int]) -- call to gracefully try to allow tx drain to occur
static mp_obj_t
i2cslave_alt_tx_drain_timeout_us(mp_uint_t n_args, const mp_obj_t* args)
{
    mp_int_t timeout_us = n_args ? mp_obj_get_int(args[0]) : 0;
    int res = slvmem_tx_drain_timeout_us(timeout_us);
    return mp_obj_new_int(res);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(i2cslave_alt_tx_drain_timeout_us_obj, 0, 1, i2cslave_alt_tx_drain_timeout_us);


// This API exists to allow Python to build an immediate wake-up mainloop, a timeout_us
// value of zero will not block only check and reset any outstanding signalling condition

// .get_notify_abs(): int
static mp_obj_t
i2cslave_alt_get_notify_abs(void)
{
    uint64_t res = slvmem_get_notify_abs();
    return mp_obj_new_int_from_ll(res); // 64bit on 32bit platform
}
static MP_DEFINE_CONST_FUN_OBJ_0(i2cslave_alt_get_notify_abs_obj, i2cslave_alt_get_notify_abs);


// allows to sleep waiting for I2C activity, takes timeout_abs in absolute deadline
// .waitfor_notify_timeout_abs(notify_abs:int, [timeout_abs:int])
static mp_obj_t
i2cslave_alt_waitfor_notify_timeout_abs(mp_uint_t n_args, const mp_obj_t* args)
{
    // both notify_abs and timeout_abs are 64bit integers
    uint64_t notify_abs  = (n_args > 0 && mp_obj_is_integer(args[0])) ? mp_obj_int_get_checked(args[0]) : 0;
    uint64_t timeout_abs = (n_args > 1 && mp_obj_is_integer(args[1])) ? mp_obj_int_get_checked(args[1]) : 0;
    uint64_t res = slvmem_waitfor_notify_timeout_abs(notify_abs, timeout_abs);
    return mp_obj_new_int_from_ll(res); // 64bit on 32bit platform
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(i2cslave_alt_waitfor_notify_timeout_abs_obj, 1, 2, i2cslave_alt_waitfor_notify_timeout_abs);


// allows to sleep waiting for I2C activity, takes timeout_us as relative micro-seconds
// .waitfor_notify_timeout_us(notify_abs:int, [timeout_us:int])
static mp_obj_t
i2cslave_alt_waitfor_notify_timeout_us(mp_uint_t n_args, const mp_obj_t* args)
{
    // notify_abs is a 64bit integers
    uint64_t notify_abs = (n_args > 0 && mp_obj_is_integer(args[0])) ? mp_obj_int_get_checked(args[0]) : 0;
    uint64_t timeout_us = (n_args > 1) ? mp_obj_get_int(args[1]) : 0;
    uint64_t res = slvmem_waitfor_notify_timeout_us(notify_abs, timeout_us);
    return mp_obj_new_int_from_ll(res); // 64bit on 32bit platform
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(i2cslave_alt_waitfor_notify_timeout_us_obj, 1, 2, i2cslave_alt_waitfor_notify_timeout_us);


// .tx_available_raw(): int
static mp_obj_t
i2cslave_alt_tx_available_raw(void)
{
    // the available to read count is the same as outstanding when looking from the opposite side
    size_t value = xfer_buffer_tx_avail_raw();
    return mp_obj_new_int(value);
}
static MP_DEFINE_CONST_FUN_OBJ_0(i2cslave_alt_tx_available_raw_obj, i2cslave_alt_tx_available_raw);


// .tx_space_raw(): int The remaining space to take new data
static mp_obj_t
i2cslave_alt_tx_space_raw(void)
{
    size_t value = xfer_buffer_tx_space_raw();
    return mp_obj_new_int(value);
}
static MP_DEFINE_CONST_FUN_OBJ_0(i2cslave_alt_tx_space_raw_obj, i2cslave_alt_tx_space_raw);


// This is an API to replace data_out_done_callback, it returns an integer count of the
//  number of bytes not yet cleared over I2C so it returns zero when done (idle).
// Use of i2cslave_alt_tx_drain_timeout_us(timeout_us) is better suited to a graceful wait
//  for flush with a limited-wait to achieve the condition of tx_available()==0.
// .tx_available(): int
static mp_obj_t
i2cslave_alt_tx_available(void)
{
    // the available to read count is the same as outstanding when looking from the opposite side
    size_t value = xfer_buffer_tx_avail();
    return mp_obj_new_int(value);
}
static MP_DEFINE_CONST_FUN_OBJ_0(i2cslave_alt_tx_available_obj, i2cslave_alt_tx_available);


// .tx_space(): int The remaining space to take new data
static mp_obj_t
i2cslave_alt_tx_space(void)
{
    size_t value = xfer_buffer_tx_space();
    return mp_obj_new_int(value);
}
static MP_DEFINE_CONST_FUN_OBJ_0(i2cslave_alt_tx_space_obj, i2cslave_alt_tx_space);


// .tx_capacity(): int The total capacity of the buffer
static mp_obj_t
i2cslave_alt_tx_capacity(void)
{
    size_t value = xfer_buffer_tx_capacity();
    return mp_obj_new_int(value);
}
static MP_DEFINE_CONST_FUN_OBJ_0(i2cslave_alt_tx_capacity_obj, i2cslave_alt_tx_capacity);


// .tx_granularity(): int The granularity of the data packets
static mp_obj_t
i2cslave_alt_tx_granularity(void)
{
    size_t value = xfer_buffer_tx_granularity();
    return mp_obj_new_int(value);
}
static MP_DEFINE_CONST_FUN_OBJ_0(i2cslave_alt_tx_granularity_obj, i2cslave_alt_tx_granularity);


// .rx_available_raw(): int The amount of data waiting to be read
static mp_obj_t
i2cslave_alt_rx_available_raw(void)
{
    size_t avail = xfer_buffer_rx_avail_raw();
    return mp_obj_new_int(avail);
}
static MP_DEFINE_CONST_FUN_OBJ_0(i2cslave_alt_rx_available_raw_obj, i2cslave_alt_rx_available_raw);


// .rx_space_raw(): int The amount of empty space the I2C master can use to provide more data
static mp_obj_t
i2cslave_alt_rx_space_raw(void)
{
    size_t value = xfer_buffer_rx_space_raw();
    return mp_obj_new_int(value);
}
static MP_DEFINE_CONST_FUN_OBJ_0(i2cslave_alt_rx_space_raw_obj, i2cslave_alt_rx_space_raw);


// .rx_available(): int The amount of data waiting to be read
static mp_obj_t
i2cslave_alt_rx_available(void)
{
    size_t avail = xfer_buffer_rx_avail();
    return mp_obj_new_int(avail);
}
static MP_DEFINE_CONST_FUN_OBJ_0(i2cslave_alt_rx_available_obj, i2cslave_alt_rx_available);


// .rx_space(): int The amount of empty space the I2C master can use to provide more data
static mp_obj_t
i2cslave_alt_rx_space(void)
{
    size_t value = xfer_buffer_rx_space();
    return mp_obj_new_int(value);
}
static MP_DEFINE_CONST_FUN_OBJ_0(i2cslave_alt_rx_space_obj, i2cslave_alt_rx_space);


// .rx_capacity(): int The total capacity of the buffer
static mp_obj_t
i2cslave_alt_rx_capacity(void)
{
    size_t value = xfer_buffer_rx_capacity();
    return mp_obj_new_int(value);
}
static MP_DEFINE_CONST_FUN_OBJ_0(i2cslave_alt_rx_capacity_obj, i2cslave_alt_rx_capacity);


// .rx_granularity(): int The granularity of the data packets
static mp_obj_t
i2cslave_alt_rx_granularity(void)
{
    size_t value = xfer_buffer_rx_granularity();
    return mp_obj_new_int(value);
}
static MP_DEFINE_CONST_FUN_OBJ_0(i2cslave_alt_rx_granularity_obj, i2cslave_alt_rx_granularity);


// .get_busy_state(): int The granularity of the data packets
static mp_obj_t
i2cslave_alt_get_busy_state(void)
{
    size_t value = slvmem_get_busy_state();
    return mp_obj_new_int(value);
}
static MP_DEFINE_CONST_FUN_OBJ_0(i2cslave_alt_get_busy_state_obj, i2cslave_alt_get_busy_state);


// ensure Deinit state is achieved
// .deinitialize([init_mode:int=DEFAULT]) // caller needs to supply FORCE value 0xff
static mp_obj_t
i2cslave_alt_deinitialize(mp_uint_t n_args, const mp_obj_t* args)
{
    uint32_t init_mode = (n_args > 0) ? mp_obj_get_int(args[0]) : INIT_M_DEFAULT;
    return slvmem_i2c_deinitialize(init_mode) ? mp_const_true : mp_const_false;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(i2cslave_alt_deinitialize_obj, 0, 1, i2cslave_alt_deinitialize);


// setup function to specify params, stores configuration data from Python into C.
// .setup(address, scl, sda, baud, [use_pullups])
static mp_obj_t
i2cslave_alt_setup(mp_uint_t n_args, const mp_obj_t* args)
{
    mp_int_t address  = mp_obj_get_int(args[0]);
    mp_int_t scl_pin  = mp_obj_get_int(args[1]);
    mp_int_t sda_pin  = mp_obj_get_int(args[2]);
    mp_int_t baudrate = mp_obj_get_int(args[3]);
    mp_int_t pullups  = (n_args > 4) ? mp_obj_get_int(args[4]) : i2c_use_pullups;

    // Validate parameters (I2C protocol does not allow address zero for SLAVE use)
    if(address < 1 || address > 127)
        mp_raise_ValueError(MP_ERROR_TEXT("address must be 1-127"));
    i2c_address     = (uint8_t)address;

    if(scl_pin < 0 || scl_pin > 29 || sda_pin < 0 || sda_pin > 29)
        mp_raise_ValueError(MP_ERROR_TEXT("pins must be 0-29"));
    if(scl_pin == sda_pin)
        mp_raise_ValueError(MP_ERROR_TEXT("SCL and SDA pins must be different"));
    i2c_pin_sda     = (uint8_t)sda_pin;
    i2c_pin_scl     = (uint8_t)scl_pin;

    i2c_baudrate    = baudrate;
    i2c_use_pullups = (uint8_t)pullups;

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(i2cslave_alt_setup_obj, 4, 5, i2cslave_alt_setup);


// Write bytes: queue data out to return on reads from master
// .tx_write_bytes(bytes, offset, len, [minwrite:int = 0])
static mp_obj_t
i2cslave_alt_tx_write_bytes(mp_uint_t n_args, const mp_obj_t* args)
{
    // Extract bts (byte array)
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[0], &bufinfo, MP_BUFFER_READ);

    int offset   = n_args > 1 ? mp_obj_get_int(args[1]) : 0;
    int len      = n_args > 2 ? mp_obj_get_int(args[2]) : bufinfo.len - offset;
    int minwrite = n_args > 3 ? mp_obj_get_int(args[3]) : 0;
    if(offset < 0 || len < 0 || minwrite < 0)
        mp_raise_ValueError(MP_ERROR_TEXT("invalid arguments"));

    // The extra checks seem superfluious but what if offset+len is unsigned overflow
    if(bufinfo.len < (offset + len) || bufinfo.len < offset || bufinfo.len < len)
        mp_raise_ValueError(MP_ERROR_TEXT("buffer too small"));

    const uint8_t* buf  = (const uint8_t*)bufinfo.buf; // cast to const byte*
    const uint8_t* data = &buf[offset];
    int res = slvmem_tx_write_bytes(data, len, minwrite);
    return mp_obj_new_int(res);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(i2cslave_alt_tx_write_bytes_obj, 1, 4, i2cslave_alt_tx_write_bytes);


static mp_obj_t
i2cslave_alt_tx_trylock_timeout_us(mp_obj_t o_timeout_us)
{
    uint32_t timeout_us = mp_obj_get_int(o_timeout_us);
    int res = slvmem_tx_trylock_timeout_us(timeout_us);
    return mp_obj_new_int(res);
}
static MP_DEFINE_CONST_FUN_OBJ_1(i2cslave_alt_tx_trylock_timeout_us_obj, i2cslave_alt_tx_trylock_timeout_us);


static mp_obj_t
i2cslave_alt_tx_unlock(void)
{
    slvmem_tx_unlock();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(i2cslave_alt_tx_unlock_obj, i2cslave_alt_tx_unlock);


static mp_obj_t
i2cslave_alt_rx_trylock_timeout_us(mp_obj_t o_timeout_us)
{
    uint32_t timeout_us = mp_obj_get_int(o_timeout_us);
    int res = slvmem_rx_trylock_timeout_us(timeout_us);
    return mp_obj_new_int(res);
}
static MP_DEFINE_CONST_FUN_OBJ_1(i2cslave_alt_rx_trylock_timeout_us_obj, i2cslave_alt_rx_trylock_timeout_us);


static mp_obj_t
i2cslave_alt_rx_unlock(void)
{
    slvmem_rx_unlock();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(i2cslave_alt_rx_unlock_obj, i2cslave_alt_rx_unlock);


// Simulate some kind of potential low-level handling, this is non-API work, non-ISR handler work
static atomic_short driver_stuff_int;
static mp_obj_t
i2cslave_alt_do_driver_stuff(void)
{
    int res = driver_stuff_int++;
    return mp_obj_new_int(res);
}
static MP_DEFINE_CONST_FUN_OBJ_0(i2cslave_alt_do_driver_stuff_obj, i2cslave_alt_do_driver_stuff);


// Define the module’s attribute table
static const mp_rom_map_elem_t i2cslave_alt_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),                    MP_ROM_QSTR(MP_QSTR_i2cslave_alt) },
    { MP_ROM_QSTR(MP_QSTR_setup),                       MP_ROM_PTR(&i2cslave_alt_setup_obj) },
    { MP_ROM_QSTR(MP_QSTR_initialize),                  MP_ROM_PTR(&i2cslave_alt_initialize_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinitialize),                MP_ROM_PTR(&i2cslave_alt_deinitialize_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_busy_state),              MP_ROM_PTR(&i2cslave_alt_get_busy_state_obj) },

    { MP_ROM_QSTR(MP_QSTR_get_notify_abs),              MP_ROM_PTR(&i2cslave_alt_get_notify_abs_obj) },
    { MP_ROM_QSTR(MP_QSTR_waitfor_notify_timeout_abs),  MP_ROM_PTR(&i2cslave_alt_waitfor_notify_timeout_abs_obj) },
    { MP_ROM_QSTR(MP_QSTR_waitfor_notify_timeout_us),   MP_ROM_PTR(&i2cslave_alt_waitfor_notify_timeout_us_obj) },

    { MP_ROM_QSTR(MP_QSTR_rx_peek_bytes),               MP_ROM_PTR(&i2cslave_alt_rx_peek_bytes_obj) },
    { MP_ROM_QSTR(MP_QSTR_rx_read_bytes),               MP_ROM_PTR(&i2cslave_alt_rx_read_bytes_obj) },

    { MP_ROM_QSTR(MP_QSTR_tx_write_bytes),              MP_ROM_PTR(&i2cslave_alt_tx_write_bytes_obj) },
    { MP_ROM_QSTR(MP_QSTR_tx_discard_all),              MP_ROM_PTR(&i2cslave_alt_tx_discard_all_obj) },
    { MP_ROM_QSTR(MP_QSTR_tx_drain_timeout_abs),        MP_ROM_PTR(&i2cslave_alt_tx_drain_timeout_abs_obj) },
    { MP_ROM_QSTR(MP_QSTR_tx_drain_timeout_us),         MP_ROM_PTR(&i2cslave_alt_tx_drain_timeout_us_obj) },

    { MP_ROM_QSTR(MP_QSTR_tx_available_raw),            MP_ROM_PTR(&i2cslave_alt_tx_available_raw_obj) },
    { MP_ROM_QSTR(MP_QSTR_tx_space_raw),                MP_ROM_PTR(&i2cslave_alt_tx_space_raw_obj) },
    { MP_ROM_QSTR(MP_QSTR_tx_available),                MP_ROM_PTR(&i2cslave_alt_tx_available_obj) },
    { MP_ROM_QSTR(MP_QSTR_tx_space),                    MP_ROM_PTR(&i2cslave_alt_tx_space_obj) },
    { MP_ROM_QSTR(MP_QSTR_tx_capacity),                 MP_ROM_PTR(&i2cslave_alt_tx_capacity_obj) },
    { MP_ROM_QSTR(MP_QSTR_tx_granularity),              MP_ROM_PTR(&i2cslave_alt_tx_granularity_obj) },

    { MP_ROM_QSTR(MP_QSTR_rx_available_raw),            MP_ROM_PTR(&i2cslave_alt_rx_available_raw_obj) },
    { MP_ROM_QSTR(MP_QSTR_rx_space_raw),                MP_ROM_PTR(&i2cslave_alt_rx_space_raw_obj) },
    { MP_ROM_QSTR(MP_QSTR_rx_available),                MP_ROM_PTR(&i2cslave_alt_rx_available_obj) },
    { MP_ROM_QSTR(MP_QSTR_rx_space),                    MP_ROM_PTR(&i2cslave_alt_rx_space_obj) },
    { MP_ROM_QSTR(MP_QSTR_rx_capacity),                 MP_ROM_PTR(&i2cslave_alt_rx_capacity_obj) },
    { MP_ROM_QSTR(MP_QSTR_rx_granularity),              MP_ROM_PTR(&i2cslave_alt_rx_granularity_obj) },

    { MP_ROM_QSTR(MP_QSTR_tx_trylock_timeout_us),       MP_ROM_PTR(&i2cslave_alt_tx_trylock_timeout_us_obj) },
    { MP_ROM_QSTR(MP_QSTR_tx_unlock),                   MP_ROM_PTR(&i2cslave_alt_tx_unlock_obj) },

    { MP_ROM_QSTR(MP_QSTR_rx_trylock_timeout_us),       MP_ROM_PTR(&i2cslave_alt_rx_trylock_timeout_us_obj) },
    { MP_ROM_QSTR(MP_QSTR_rx_unlock),                   MP_ROM_PTR(&i2cslave_alt_rx_unlock_obj) },

    { MP_ROM_QSTR(MP_QSTR_do_driver_stuff),             MP_ROM_PTR(&i2cslave_alt_do_driver_stuff_obj) }
};
static MP_DEFINE_CONST_DICT(i2cslave_alt_module_globals, i2cslave_alt_module_globals_table);

// Define the module object
const mp_obj_module_t i2cslave_alt_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&i2cslave_alt_module_globals,
};

// Register the module
MP_REGISTER_MODULE(MP_QSTR_i2cslave_alt, i2cslave_alt_module);
