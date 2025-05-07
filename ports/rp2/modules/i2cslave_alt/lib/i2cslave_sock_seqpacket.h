/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2025 Pat Deegan, https://psychogenic.com
 * Copyright (c) 2025 Darryl L. Miles
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
 *
 * SPDX-License-Identifier: MIT
 *
 */

#ifndef __I2CSLAVE_SOCK_SEQPACKET_H
#define __I2CSLAVE_SOCK_SEQPACKET_H

// The I2C master has fixed or maximum length in data exchange each direction
// The low level driver and buffering operates in a sequenced-packet mode so
//  each I2C transaction consumes buffer space in units of the packet len.
#define I2CSLAVE_RX_PACKET_LEN (8)
#define I2CSLAVE_TX_PACKET_LEN (16)

// Assumes MAX() will be provided at point of use
#define I2CSLAVE_BIGEST_PACKET_LEN (MAX(I2CSLAVE_RX_PACKET_LEN, I2CSLAVE_TX_PACKET_LEN))

// 4 packets (of 8 bytes) inbound
#define I2CSLAVE_RX_MEMBUF_LEN (I2CSLAVE_RX_PACKET_LEN*4)
// 8 packets (of 16 bytes) outbound
#define I2CSLAVE_TX_MEMBUF_LEN (I2CSLAVE_TX_PACKET_LEN*8)

#define I2CSLAVE_DEVICE i2c1

#ifdef __cplusplus
// This should only be used for GoogleTest not for production
#include <atomic>
typedef std::atomic_uchar atomic_uchar_t;
typedef std::atomic_ushort atomic_ushort_t;
#ifdef FIXME_SOMETHING_RELATING_TO_PICO_TARGET
#error "Not for production use"
#endif
#else
#include <stdatomic.h>   // C11
typedef atomic_uchar atomic_uchar_t;
typedef atomic_ushort atomic_ushort_t;
#endif

// This is written like this as 8-bit accounting data type may be too limited if MEMBUF_LEN is increased
#if (I2CSLAVE_TX_MEMBUF_LEN <= 256 && I2CSLAVE_RX_MEMBUF_LEN <= 256)
/* 8bit mode, 256 byte buffer maximum */
typedef uint8_t uintpos_t;
typedef uint16_t uintlen_t;
typedef atomic_uchar_t atomic_uintpos_t;
typedef atomic_ushort_t atomic_uintlen_t; // needs range 0..256 which is 9bits minimum
#elif (I2CSLAVE_TX_MEMBUF_LEN <= 65535 && I2CSLAVE_RX_MEMBUF_LEN <= 65535)
/* 16bit mode, 65535 byte buffer maximum, note the (2^16)-1 difference from 8bit mode */
typedef uint16_t uintpos_t;
typedef uint16_t uintlen_t;
typedef atomic_ushort_t atomic_uintpos_t;
typedef atomic_ushort_t atomic_uintlen_t; // careful range 0..65535 (not 65536) so still inside 16bits minimum
#else
 /* if you want this then maybe consider mem_len can be 1-bit that means empty-or-full and uintpos_t is 32bit */
 #error "I2CSLAVE_TX_MEMBUF_LEN || I2CSLAVE_RX_MEMBUF_LEN exceeds tested limits"
#endif

// This the dynamic bit for the ring buffer accounting
typedef struct xfer_buffer_state
{
    atomic_uintpos_t mem_in;
    atomic_uintpos_t mem_out;
    atomic_uintlen_t mem_len;
} xfer_buffer_state_t;

// This is the constant part which might run from flash
typedef struct xfer_buffer
{
    uint8_t* const              mem;
    struct xfer_buffer_state*   state;
    const uintlen_t             capacity;
    const uintlen_t             granularity;   // aka packet_len
    const uint8_t               granularity_pow2;  // 1 << granularity_pow2 == granularity
} xfer_buffer_t;


// mem_len_dir values
#define MEM_LEN_NOCHANGE 0
#define MEM_LEN_ADDITION 1
#define MEM_LEN_SUBTRACT -1

struct recv_mem_operation {
    uint8_t*        data;
    uintlen_t       len;
    uintlen_t       skip;
    int8_t          mem_len_dir;  // direction
    bool            zero_pad;
};

struct send_mem_operation {
    const uint8_t*  data;
    uintlen_t 	    len;
    int8_t          mem_len_dir; // direction
    bool            zero_pad;
};



#ifdef __cplusplus
extern "C" {
#endif

/* used for init_mode, when unsure just use INIT_M_DEFAULT */
/* the 0x0f split is saying 4bits on by default, 4bits off by default leaving space */
#define INIT_M_DEFAULT  0x0f
#define INIT_M_FORCE    0xff

#define INIT_B_LOCKS    0x01
#define INIT_B_TX       0x02
#define INIT_B_RX       0x04
#define INIT_B_FORCE    0x80

#define INIT_M_IS_INIT_LOCKS(x) (((x) & INIT_B_LOCKS) != 0)
#define INIT_M_IS_INIT_RX(x)    (((x) & INIT_B_RX) != 0)
#define INIT_M_IS_INIT_TX(x)    (((x) & INIT_B_TX) != 0)
#define INIT_M_IS_FORCE(x)      (((x) & INIT_B_FORCE) != 0)

/* Safe to call multiple times when force==false. 
 * Use of force==false is the normal use of the API.
 *
 * API is not considered thread-safe, as technically a 2nd CPU would need to
 *  ensure the 1st CPU has completed successfully before proceeding to use
 *  use other API dependent on that success.
 */
extern bool slvmem_i2c_deinitialize(uint32_t init_mode);


/* This API is useful to allow user-space to watchdog the health of the low level I2C driver.
 * This will indicate what the low-level drive is upto.
 * API is thread-safe at all times.
 */
extern int slvmem_get_busy_state(void);


/* This API is useful to allow user-space to watchdog the health of the low level I2C driver.
 * This value will be a compatible get_absolute_time() via type absolute_time_t as
 *  per PICO SDK of the last time the I2S_HANDLER_FINISIH was called, indicating some
 *  kind of I2C activity.
 * So if this valud does not seem to change it would be possible to consider a
 *  driver or hardware issue and user-space might wish to try a hardware reset
 *  maybe deinit/init or system restart.
 * API is thread-safe at all times.
 */
extern absolute_time_t slvmem_get_notify_abs(void);


/*
 * FIXME consider absolute_time_t slvmem_waitfor_notify_timeout_abs(absolute_time_t notify_abs, absolute_time_t timeout_abs)
 *   this simplifies the high-level missed-wakeup concern (by pushing the magic down here), by only allowing the block if notify_abs==slvmem_get_i2c_notify_abs()
 * API is thread-safe at all times.
 */
extern absolute_time_t slvmem_waitfor_notify_timeout_abs(absolute_time_t notify_abs, absolute_time_t timeout_abs);

/* FIXME consider absolute_time_t slvmem_waitfor_notify_timeout_us(absolute_time_t notify_abs, uint32_t timeout_us)
 * This allows to wait for a new I2C event, to attempt to allow a mainloop to sleep, hopefully in a lower-power mode.
 * API is thread-safe at all times.
 */
extern absolute_time_t slvmem_waitfor_notify_timeout_us(absolute_time_t notify_abs, uint32_t timeout_us);


enum NackMode /*: uint8_t*/ { // C23 enum storage type
   NACK_M_NONE = 0,  // use as DEFAULT
   NACK_M_GPIO = 1   // EXPERIMENAL
};

// This exists due to concern with argument order errors when making API calls
// with lots of arguments that are similar/same/implicitly converable types.
// This forces all read/write of values to have the label of what the value is
//  nearby the point it is accessed, call this type safety in C.
typedef struct i2c_slave_config {
    uint8_t sda_pin;
    uint8_t scl_pin;
    uint8_t address;
    uint baudrate;
    uint8_t use_pullups;
    uint8_t nack_mode; // enum NackMode
} i2c_slave_config_t;

extern int slvmem_i2c_initialize(const i2c_slave_config_t* cfg, uint32_t init_mode);

/* Reset the TX queue in the low-level driver.  This may cause corruption of the interpreted stream
 *  of TX data as seen by the I2C master as this can cause truncate the in-progress message midflow.
 * API is thread-safe at all times.
 */
extern void slvmem_tx_discard_all(void);


/* Drain's the TX data output (meaning the data is nominally sent over the I2C from us to the master,
 *  as this is a polled interface and we don't control when polling occurs the limited timeout_us
 *  allows this to block,  this is providing a limited wait mechanism.
 * There is no wait forever setting but nothing stops you using UINT32_MAX but that is not recommended.
 * This function returns with 0 immediately if there is no data in the tx buffer to drain.
 * see xfer_buffer_tx_avail()
 * API is thread-safe at all times.
 */
extern int slvmem_tx_drain_timeout_abs(absolute_time_t timeout_abs);

/* The real API uses absolute_time_t this function is just a helper to allow use of relative timeout_us
 *  in relative micro-seconds.
 * A timeout_us=0 results in non-blocking behavior, and would report the amount of data in the tx.
 * see slvmem_tx_drain_timeout_abs(absolute_time_t timeout_abs)
 * see xfer_buffer_tx_avail()
 * API is thread-safe at all times.
 */
extern int slvmem_tx_drain_timeout_us(uint32_t timeout_us);


/* Return the number of bytes taken, these will be padded into TX data granularity I2CSLAVE_TX_PACKET_LEN
 *  for the excess data at the end of the provided 'len'. */
/* The use of minwrite allows the caller to request the low-level API is transactional.
 *  minwrite should always be a value between 0 and len.  No other value makes sense and will just cause error.
 * Via this API the caller has most control to be able to ensure consecutive packets are put into the buffer
 *  or if they don't care.
 * The data stream might be described as sequenced-packet where the data order is maintain and packeization
 *  occurs with zero-padding inserted (when the last block of data is short of granulatity I2CSLAVE_TX_PACKET_LEN).
 * Even with 'minwrite' set to zero the low-level system may still be designed to only take granularity
 *  chunks of data, so the return values seen will be in units of that granularity.
 * API should only be used after successful initialization.
 * API access MUST be serialized to make it thread-safe with itself.
 * API is not reentrant with itself.
 *  Use coperative TX locking API around calls to achieve thread-safety via serialized access or design
 *  application to ensure calls never overlap
 * For clarity it is safe to overlap calls to slvmem_rx_write_bytes() and/or
 *  slvmem_rx_peek_bytes() with this API.
 */
extern int slvmem_tx_write_bytes(const uint8_t* data, size_t len, size_t minwrite);


/* This is a non-destructive read of data in the queue.  Maybe useful to allow peeking of the protocol ID
 *  to make a decision on how to handle.
 * API should only be used after successful initialization.
 * API access SHOULD be serialized to make it thread-safe with slvmem_rx_read_bytes() use.
 * API is reentrant with itself.  Two CPUs may overlap calls to slvmem_rx_peek_bytes().
 *  Use coperative RX locking API around calls to achieve thread-safety via serialized access or design
 *  application to ensure calls never overlap (with slvmem_rx_read_bytes())
 * For clarity it is safe to overlap calls to slvmem_tx_write_bytes() with this API.
 */
extern int slvmem_rx_peek_bytes(uint8_t* dest, size_t offpeek, size_t len, size_t minpeek);

/* 'dest' maybe NULL, in which case this function will destructivly discard the data and no copy is performed.
 *  This is effectively a 'cut' operation without needing additional API.
 * Return value is the number of actual bytes copied/removed.
 * API should only be used after successful initialization.
 * API access MUST be serialized to make it thread-safe with itself.  API is not reentrant with itself.
 *  Use coperative RX locking API around calls to achieve thread-safety via serialized access or design
 *  application to ensure calls never overlap.
 * For clarity it is safe to overlap calls to slvmem_tx_write_bytes() with this API.
 */
extern int slvmem_rx_read_bytes(uint8_t* dest, size_t len, size_t minread);


/* Coperataive locking support for high-level API to manage both CPUs calling:
 *   slvmem_tx_write_bytes()
 * API is thread-safe at all times.
 */
extern int slvmem_tx_trylock_timeout_us(uint32_t timeout_us);
/*
 * API is thread-safe at all times.
 */
extern void slvmem_tx_unlock(void);

/* Cooperative locking support for high-level API to manage both CPUs calling:
 *   slvmem_rx_read_bytes()
 *   slvmem_rx_peek_bytes()
 * Note it should be safe to both both CPUs call peek at the same but to be safe
 *  participate with coperative locking.
 * API is thread-safe at all times.
 */
extern int slvmem_rx_trylock_timeout_us(uint32_t timeout_us);
/*
 * API is thread-safe at all times.
 */
extern void slvmem_rx_unlock(void);


/*
 * avail as in the available data the valid payload part.
 * API is thread-safe at all times.
 */
extern size_t xfer_buffer_tx_avail(void);
/*
 * space as in free-space the empty part.
 * API is thread-safe at all times.
 */
extern size_t xfer_buffer_tx_space(void);
/*
 * API is thread-safe at all times.
 */
extern size_t xfer_buffer_tx_capacity(void);
/*
 * API is thread-safe at all times.
 */
extern size_t xfer_buffer_tx_granularity(void);

/*
 * API is thread-safe at all times.
 */
extern size_t xfer_buffer_rx_avail(void);
/*
 * API is thread-safe at all times.
 */
extern size_t xfer_buffer_rx_space(void);
/*
 * API is thread-safe at all times.
 */
extern size_t xfer_buffer_rx_capacity(void);
/*
 * API is thread-safe at all times.
 */
extern size_t xfer_buffer_rx_granularity(void);

/*
 * user-space probably does not need to use this for normal functioning
 * the API will expose the 1-byte granularity nature of the ISR.
 * API is thread-safe at all times.
 */
extern size_t xfer_buffer_tx_avail_raw(void);
/*
 * user-space probably does not need to use this for normal functioning
 * the API will expose the 1-byte granularity nature of the ISR.
 * API is thread-safe at all times.
 */
extern size_t xfer_buffer_tx_space_raw(void);

/*
 * user-space probably does not need to use this for normal functioning
 * the API will expose the 1-byte granularity nature of the ISR.
 * API is thread-safe at all times.
 */
extern size_t xfer_buffer_rx_avail_raw(void);
/*
 * user-space probably does not need to use this for normal functioning
 * the API will expose the 1-byte granularity nature of the ISR.
 * API is thread-safe at all times.
 */
extern size_t xfer_buffer_rx_space_raw(void);


#ifdef __cplusplus
}
#endif

#endif // __I2CSLAVE_SOCK_SEQPACKET_H
