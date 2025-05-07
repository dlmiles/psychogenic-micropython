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

#ifdef TEST_WITH_FAKE_RP2040
#include <stdlib.h>      // rand()
#include <fake_rp2040_api.h>
#include "../test_debuglog.h"
#ifdef __arm__
/* just in case */
#error "ERROR: Looks lile you are trying to build for __arm__ with TEST_WITH_FAKE_RP2040 defined"
#endif
#else
#include <hardware/i2c.h>
#include <pico/i2c_slave.h>
#include <pico/stdlib.h>
#include <pico/sync.h>
#include <pico/multicore.h>
#endif
#include <assert.h>
#include <stdatomic.h>   // C11
#include <limits.h>      // UCHAR_MAX
#include <stdio.h>
#include <string.h>

#include "./i2cslave_sock_seqpacket.h"
#include "./i2cslave_sock_seqpacket_private.h"

#ifndef TEST_WITH_FAKE_RP2040

/* copied from "test_debuglog.h" as I don't actually want to include that file for production building */
#define DEBUG_ENTRY(fmt, ...) do {} while(0)
#define DEBUG_RETURN(fmt, ...) do {} while(0)
#define DEBUG_MESSAGE(fmt, ...) do {} while(0)

#define DEBUG_ASSERT(cond)  /* */
#define DEBUG_FILL(ptr, len) /* */

#endif

#ifndef MAX
// Looks like RP2040 C/C++ SDK provides a version of MAX()

// Careful with C lack of stricter type safety, these are unsigned.
static inline size_t
MAX(size_t a, size_t b)
{
    return (a > b) ? a : b;
}
#endif

#ifndef MIN
// Looks like RP2040 C/C++ SDK provides a version of MIN()
static inline size_t
MIN(size_t a, size_t b)
{
    return (a > b) ? b : a;
}
#endif

static spin_lock_t* lockid = NULL;

static void
i2c_slave_lock_init(void)
{
    // due to the ISR use of this locking, it needs to be allocated independantly of anything else
    // this is because in normal mode with interrupts enabled the ISR (which is pinned to CPU that
    // called i2c_init()) can preempt the CPU when a spinlock is held.
    if(lockid == NULL) {
        int lock_num = spin_lock_claim_unused(false); // false to prevent panic
        if(lock_num < 0) {
            // if we get here, we are running on hope
            lock_num = next_striped_spin_lock_num(); // best-effort but shared
            printf("WARNING: spin_lock_claim_unused() failed: lock_num=%d\n", lock_num);
        }
        lockid = spin_lock_init(lock_num);
    }
}

// This puts a label on the function
static inline bool
timeout_abs_is_lt(absolute_time_t a, absolute_time_t b)
{
    return a < b;
}

static inline bool
timeout_abs_is_lt_now(absolute_time_t timeout_abs) // aka timeout_abs_has_expired()
{
    return timeout_abs_is_lt(timeout_abs, get_absolute_time());
}

// This arrangement is so we can have a dynamic part and a const part,
//  minimize RAM use and maximize immutability, maximize code reuse
//  (even with memory buffer length differences)
static xfer_buffer_state_t rx_context_state;
static uint8_t rx_context_mem[I2CSLAVE_RX_MEMBUF_LEN];

static const xfer_buffer_t rx_context = {
    .mem = rx_context_mem,
    .state = &rx_context_state,
    .capacity = (uintlen_t)sizeof(rx_context_mem), // capacity
    .granularity = I2CSLAVE_RX_PACKET_LEN,
    .granularity_pow2 = 3 // 8
};

static xfer_buffer_state_t tx_context_state;
static uint8_t tx_context_mem[I2CSLAVE_TX_MEMBUF_LEN];

static const xfer_buffer_t tx_context = {
    .mem = tx_context_mem,
    .state = &tx_context_state,
    .capacity = (uintlen_t)sizeof(tx_context_mem), // capacity
    .granularity = I2CSLAVE_TX_PACKET_LEN,
    .granularity_pow2 = 4 // 16
};

#if 0
// FIXME rewrite this intention in a form that compiles
_Static_assert(rx_context.capacity % rx_context.granularity == 0, "rx_context.capacity % rx_context.granularity == 0");
_Static_assert(1ULL << rx_context.granularity_pow2 == rx_context.granularity, "1ULL << rx_context.granularity_pow2 == rx_context.granularity");

_Static_assert(tx_context.capacity % tx_context.granularity == 0, "tx_context.capacity % tx_context.granularity == 0");
_Static_assert(1ULL << tx_context.granularity_pow2 == tx_context.granularity, "1ULL << tx_context.granularity_pow2 == tx_context.granularity");
#endif

// Sanity check ranges (this is making sure you have configured C types large enough to contain values)
_Static_assert(I2CSLAVE_RX_PACKET_LEN <= (sizeof(uintpos_t)<<8), "I2CSLAVE_RX_PACKET_LEN <= (sizeof(uintpos_t)<<8)");
_Static_assert(sizeof(rx_context_mem) <= (sizeof(uintpos_t)<<8), "sizeof(rx_context_mem) <= (sizeof(uintpos_t)<<8)");
_Static_assert(sizeof(rx_context_mem) <  (sizeof(uintlen_t)<<8), "sizeof(rx_context_mem) <= (sizeof(uintlen_t)<<8)");

_Static_assert(I2CSLAVE_TX_PACKET_LEN <= (sizeof(uintpos_t)<<8), "I2CSLAVE_TX_PACKET_LEN <= (sizeof(uintpos_t)<<8)");
_Static_assert(sizeof(tx_context_mem) <= (sizeof(uintpos_t)<<8), "sizeof(tx_context_mem) <= (sizeof(uintpos_t)<<8)");
_Static_assert(sizeof(tx_context_mem) <  (sizeof(uintlen_t)<<8), "sizeof(tx_context_mem) <  (sizeof(uintlen_t)<<8)");

// Used to indicate a low-level action occured that modified the obserable state to higher-level API
//  (usually data was received or data was transmitted).  So serves as a signal to indicate the
//  higher-level API should reevaluate the situation to see what occurred.
static atomic_ullong i2c_notify_abs; /* uint64_t */
static atomic_char i2c_notify;

// EXPERIMENTAL when not set to NACK_M_NONE, this is a feature flag provided at initialization
static uint8_t i2c_nack_mode; // enum NackMode

// Sanity check we understand the sizes
_Static_assert(sizeof(atomic_ullong) >= sizeof(absolute_time_t), "sizeof(atomic_ullong) >= sizeof(absolute_time_t)");
_Static_assert(sizeof(atomic_ullong) == sizeof(absolute_time_t), "sizeof(atomic_ullong) == sizeof(absolute_time_t)");

absolute_time_t
slvmem_get_notify_abs(void)
{
    return (absolute_time_t) atomic_load(&i2c_notify_abs); // this allows high-level API to watchdog link
}

// A copy of xfer_buffer_state_t but with capacity added
typedef struct xfer_buffer_state_tmp_t {
    uintpos_t mem_in;
    uintpos_t mem_out;
    uintlen_t mem_len;
    uintlen_t capacity;
    uintpos_t mem_pos; // temporary
} xfer_buffer_state_tmp_t;

// based on the given info compute contigious available (valid data part)
static inline size_t
compute_avail_nowrap(const xfer_buffer_state_tmp_t* st, size_t mem_out)
{
    // mem_out is provided as argument so the skiplen feature can work, so the argument
    // mem_out here will have accounted for the skip and wrapping
    if(st->mem_in == mem_out)
        return st->mem_len ? st->capacity : 0; // if mem_len==0 then empty
    return (st->mem_in >= mem_out) ?
        (size_t)(st->mem_in - mem_out) :
        (size_t)(st->capacity - mem_out); // do not add wrap-around
}

// based on the given info compute contigious space (empty part)
static inline size_t
compute_space_nowrap(const xfer_buffer_state_tmp_t* st)
{
    if(st->mem_in == st->mem_out)
        return st->mem_len ? 0 : st->capacity; // if mem_len!=0 then full
    return (st->mem_in >= st->mem_out) ?
        (size_t)(st->mem_in - st->mem_out) :
        (size_t)(st->capacity - st->mem_out); // do not add wrap-around
}

// based on the given info compute advanced mem_in by length
static inline uintpos_t
compute_advance(size_t capacity, uintpos_t mem_in, uintpos_t len)
{
    while(len >= capacity) // is this really need ?
        len -= capacity;
    // len now has modulus of capacity
    size_t nwavail = capacity - mem_in;
    if(nwavail > len) {
        // a simple advance as it does not wrap-around
        mem_in += len;
    } else {
        // we want (nwavail == len) to fallthru to here to cause wrap around to 0
        len -= nwavail;
        mem_in = len; //  wrap around reset
    }
    return mem_in;
}

void
xfer_buffer_state_init(xfer_buffer_state_t* ctxt)
{
    DEBUG_ASSERT(ctxt); // GCOV_EXCL_LINE
    // data area does not need to be zeroed
    ctxt->mem_in  = 0;
    ctxt->mem_out = 0;
    ctxt->mem_len = 0;
}

// round(8, 0) = 0
// round(8, 1) = 7
// round(8, 2) = 6
// round(8, 7) = 1
// round(8, 8) = 0
// round(8, 9) = 7
static inline size_t
compute_padlen_for_blocksize(size_t blocksize, size_t value)
{
    size_t res = value % blocksize;
    return res ? blocksize - res : 0;
}

// packet remaining, liek the above but returns blocksize for zero
// round(8, 0) = 8
// round(8, 1) = 7
// round(8, 2) = 6
// round(8, 7) = 1
// round(8, 8) = 8
// round(8, 9) = 7
static inline size_t
compute_pktrem_for_blocksize(size_t blocksize, size_t value)
{
    size_t res = value % blocksize;
    return res ? blocksize - res : blocksize;
}

// offset0 as in the first position of the current block
static inline size_t
compute_offset0_blocksize(size_t blocksize, size_t value)
{
    size_t mod = value % blocksize;
    return value - mod;
}

// blocksize_pow2 == 3 means blocksize==8
// blocksize_pow2 == 4 means blocksize==16
static inline size_t
compute_padlen_for_blocksize2(size_t blocksize_pow2, size_t value)
{
    DEBUG_ASSERT(blocksize_pow2 > 0 && blocksize_pow2 < 32); // GCOV_EXCL_LINE
    const size_t blocksize = (size_t)1 << blocksize_pow2;
    const size_t mask = blocksize - 1;
    const size_t res = value & mask;
    return res ? blocksize - res : 0;
}

// blocksize_pow2 == 3 means blocksize==8
// blocksize_pow2 == 4 means blocksize==16
static inline size_t
compute_pktrem_for_blocksize2(size_t blocksize_pow2, size_t value)
{
    DEBUG_ASSERT(blocksize_pow2 > 0 && blocksize_pow2 < 32); // GCOV_EXCL_LINE
    const size_t blocksize = (size_t)1 << blocksize_pow2;
    const size_t mask = blocksize - 1;
    const size_t res = value & mask;
    return res ? blocksize - res : blocksize;
}

// blocksize_pow2 == 3 means blocksize==8
// blocksize_pow2 == 4 means blocksize==16
static inline size_t
compute_offset0_blocksize2(size_t blocksize_pow2, size_t value)
{
    DEBUG_ASSERT(blocksize_pow2 > 0 && blocksize_pow2 < 32); // GCOV_EXCL_LINE
    const size_t blocksize = (size_t)1 << blocksize_pow2;
    const size_t mask = blocksize - 1;
    const size_t res = value & mask;
    return value - res;
}


// test assumptions needed by the code nearby
_Static_assert(MEM_LEN_NOCHANGE == 0, "MEM_LEN_NOCHANGE==0");
_Static_assert(MEM_LEN_ADDITION > 0, "MEM_LEN_ADDITION>0"); // positive
_Static_assert(MEM_LEN_SUBTRACT < 0, "MEM_LEN_SUBTRACT>0"); // negative

static inline void
padfill(uint8_t* dest, size_t padlen)
{
    DEBUG_ASSERT(dest); // GCOV_EXCL_LINE
    while(padlen-- > 0)
        *dest++ = '\0';
}

// send (copy data from user-space into driver-space)
static size_t
xfer_buffer_send_memcpy(const struct send_mem_operation* op, const xfer_buffer_t* ctxt)
{
    xfer_buffer_state_t* const state = ctxt->state;

    DEBUG_ENTRY("xfer_buffer_send_memcpy(%s, %s)", send_mem_operation_to_string(op), xfer_buffer_to_string(ctxt));

    // just used a spinlock to cop-out of working out where the memory barrier would need to be
    // and the load/store order to remove the need for the spinlock.
    // we're taking a snapshot of the state to perform the decision making process then we commit
    // any changes at the end of this function in a similar atomic way.
    xfer_buffer_state_tmp_t tmpst;
    uint32_t save = spin_lock_blocking(lockid);
    // this code here is authorative in modifying tx_context.state.mem_in in this function to make new data visible to
    //  the ISR.  The ISR does modify mem_in as well but only to skip/discard/make-empty the buffer as it consumes the data
    tmpst.mem_in  = state->mem_in;
    // tmpst.mem_len the only purpose of this value is to know if mem_in/mem_out represent an empty or full buffer
    tmpst.mem_len = state->mem_len;
    // tmpst.mem_out the only purpose of this value is to compute where the bondary or avail/space is in the buffer
    tmpst.mem_out = state->mem_out;
    spin_unlock(lockid, save);
    tmpst.capacity = ctxt->capacity; // copied from const data

    const uint8_t* src = op->data;
    size_t left = op->len;
    size_t n = 0;

    // disallow MEM_LEN_SUBTRACT (it is not used in the ISR path at this time) so is not tested
    DEBUG_ASSERT(op->mem_len_dir >= 0); // GCOV_EXCL_LINE

    // the low level ISR handler can not see the data payload bytes until we update account

    if(left > 0) { // src < endp
        size_t nwspace = compute_space_nowrap(&tmpst); // no-wrap-space (contigious space)
        size_t copylen = MIN(nwspace, left);
        uint8_t* dest = &ctxt->mem[(unsigned)tmpst.mem_in];
        if(src) {
            memcpy(dest, src, copylen); // memcpy_advance()
            src += copylen;
        } else {
            // not sure what use this is, but better than sending random
            padfill(dest, copylen);
        }
        n += copylen;
        left -= copylen;
        tmpst.mem_in = compute_advance(ctxt->capacity, tmpst.mem_in, copylen); // fixup wrap around
    }

    if(left > 0) {  // basically a loop unroll of the same as above (so we don't loop)
        size_t nwspace = compute_space_nowrap(&tmpst); // no-wrap-space
        size_t copylen = MIN(nwspace, left);
        uint8_t* dest = &ctxt->mem[(unsigned)tmpst.mem_in];
        if(src) {
            memcpy(dest, src, copylen); // memcpy_advance()
            src += copylen;
        } else {
            // not sure what use this is, but better than sending random
            padfill(dest, copylen);
        }
        n += copylen;
        left -= copylen;
        tmpst.mem_in = compute_advance(ctxt->capacity, tmpst.mem_in, copylen); // fixup wrap around
    }

    const size_t padlen = compute_padlen_for_blocksize2(ctxt->granularity_pow2, n);
#ifdef TEST_WITH_FAKE_RP2040
    const size_t padlen2 = compute_padlen_for_blocksize2(ctxt->granularity == 8 ? 3 : 4, n);
    const size_t padlen3 = compute_padlen_for_blocksize(ctxt->granularity, n);
    DEBUG_ASSERT(padlen == padlen2); // GCOV_EXCL_LINE // FIXME switch the math to ensure shift-and-mask not divide-for-mod
    DEBUG_ASSERT(padlen == padlen3); // GCOV_EXCL_LINE
#endif
    if(op->zero_pad) { // fill with zero pad
        uint8_t* dest = &ctxt->mem[(unsigned)tmpst.mem_in];
        padfill(dest, padlen);
        // if we didn't add zero_pad here mem_in would be advanced over whatever data was still in the buffer (emitting garbage)
    }
    // always advance padlen so we leave it aligned and pre-wraped
    tmpst.mem_in = compute_advance(ctxt->capacity, tmpst.mem_in, padlen);

    // transactionally commit new accounting (this makes data visible to ISR)
    if(op->mem_len_dir != MEM_LEN_NOCHANGE) {
        // accounting is packet/block based, API is byte based, so we need to keep n_with_pad separate
        //  from 'n' because that is our function return value.
        uintlen_t n_with_pad = n + padlen;

        // +1 is positive and means we add (user-space uses this)
        // -1 is negative and means we subtract (ISR driver-space uses this)
        uint32_t save = spin_lock_blocking(lockid);
        // mem_in update is read-modify-write written this way to try and allow compiler (the freedom)
        //  to use atomic load/store only and not generate more locking for the addition operation
        //  which is not supported by RP2040 CPU.
        // mem_len important to reload this value during read-modify-write here, as it may have
        //   changed under us (the only purpose of tmpst.mem_len was to know if the tmpst snapshot
        //   indicates an empty or full buffer) during the decision making process above
        // mem_out may also have changed under us here, note we don't touch it anyway
        uintlen_t mem_len = state->mem_len; // implicit atomic load (into local temp) - reload
        DEBUG_ASSERT(op->mem_len_dir != 0); // GCOV_EXCL_LINE
        mem_len = (op->mem_len_dir >= 0) ? mem_len + n_with_pad : mem_len - n_with_pad;
        state->mem_in  = tmpst.mem_in;  // implicit atomic store
        state->mem_len = mem_len; // implicit atomic store
        spin_unlock(lockid, save);
    }

    DEBUG_RETURN("xfer_buffer_send_memcpy(%s, %s) = %ld", send_mem_operation_to_string(op), xfer_buffer_to_string(ctxt), n);

    return n;
}

// recv (copy data from driver-space into user-space)
static size_t
xfer_buffer_recv_memcpy(const struct recv_mem_operation* op, const xfer_buffer_t* ctxt)
{
    xfer_buffer_state_t* const state = ctxt->state;

    DEBUG_ENTRY("xfer_buffer_recv_memcpy(%s, %s)", recv_mem_operation_to_string(op), xfer_buffer_to_string(ctxt));

    // just used a spinlock to cop-out of working out where the memory barrier would need to be
    // and the load/store order to remove the need for the spinlock.
    // we're taking a snapshot of the state to perform the decision making process then we commit
    // any changes at the end of this function in a similar atomic way.
    xfer_buffer_state_tmp_t tmpst;
    uint32_t save = spin_lock_blocking(lockid);
    DEBUG_ASSERT(assert_state(ctxt)); // GCOV_EXCL_LINE
    // tmpst.mem_in the only purpose of this value is to compute where the boundary of avail/space is in the buffer
    tmpst.mem_in  = state->mem_in;
    // this code here is authorative in modifying rx_context.state.mem_out in this function to consume data to make
    //  skip/discard/make-empty the buffer.  The ISR does modify mem_out as well to make visible new data.
    tmpst.mem_out = state->mem_out;
    // tmpst.mem_len the only purpose of this value is to know if mem_in/mem_out represent an empty or full buffer
    tmpst.mem_len = state->mem_len;
    spin_unlock(lockid, save);
    tmpst.capacity = ctxt->capacity; // copied from const data
    // mem_out for SUBTRACT(rx_bytes)/NOCHANGE(peek) is working with mem_out position to consume data
    // mem_in  for ADDITION(ISR) is working with mem_in position to provide data
    tmpst.mem_pos = (op->mem_len_dir <= 0) ? tmpst.mem_out : tmpst.mem_in;

    uintlen_t skipleft = op->skip;
    // disallow feature request when MEM_LEN_ADDITION
    DEBUG_ASSERT(skipleft == 0 || op->mem_len_dir <= 0); // GCOV_EXCL_LINE
    if(skipleft > 0) {
        if(skipleft > 0) {
            size_t nwavail = compute_avail_nowrap(&tmpst, tmpst.mem_pos); // no-wrap-available (out from pos)
            size_t skiplen = MIN(nwavail, skipleft);
            skipleft -= skiplen;
            tmpst.mem_len -= skiplen;
            tmpst.mem_pos = compute_advance(ctxt->capacity, tmpst.mem_pos, skiplen); // fixup wrap around
        }
        if(skipleft > 0) { // basically a loop unroll of the same as above (so we don't loop)
            size_t nwavail = compute_avail_nowrap(&tmpst, tmpst.mem_pos); // no-wrap-available (out from pos)
            size_t skiplen = MIN(nwavail, skipleft);
            skipleft -= skiplen;
            tmpst.mem_len -= skiplen;
            tmpst.mem_pos = compute_advance(ctxt->capacity, tmpst.mem_pos, skiplen); // fixup wrap around
        }
        DEBUG_ASSERT(op->mem_len_dir == MEM_LEN_NOCHANGE); // GCOV_EXCL_LINE
        /* did not really want to modify mem_out here, but it is needed to make compute_avail_nowrap() work */
        tmpst.mem_out = tmpst.mem_pos;
        /* as this time tmpst is in a consistent state but has skipped */
    }

    uint8_t* ptr = op->data;
    size_t left = op->len; // left as in how many bytes left to fill request
    size_t n = 0;

    if(left > 0) {
        size_t nowrlen = (op->mem_len_dir <= 0) ? compute_avail_nowrap(&tmpst, tmpst.mem_pos) : // no-wrap-available (mem_out from mem_pos after skip)
                                                  compute_space_nowrap(&tmpst);  // no-wrap-space when MEM_LEN_ADDITION
        size_t copylen = MIN(nowrlen, left);
        uint8_t* memptr = &ctxt->mem[(unsigned)tmpst.mem_pos];
        if(ptr) {
            if(op->mem_len_dir <= 0)
                memcpy(ptr, memptr, copylen); // memcpy_advance()
            else
                memcpy(memptr, ptr, copylen); // memcpy_advance()
            ptr += copylen;
        }
        n += copylen;
        left -= copylen;
        tmpst.mem_len -= copylen; // have to update this so compute_????_nowrap works when op->skip>0
        tmpst.mem_pos = compute_advance(ctxt->capacity, tmpst.mem_pos, copylen); // fixup wrap around
    }

    if(left > 0) {  // basically a loop unroll of the same as above (so we don't loop)
        size_t nowrlen = (op->mem_len_dir <= 0) ? compute_avail_nowrap(&tmpst, tmpst.mem_pos) : // no-wrap-available (mem_out from mem_pos after skip)
                                                  compute_space_nowrap(&tmpst);  // no-wrap-space when MEM_LEN_ADDITION
        size_t copylen = MIN(nowrlen, left);
        uint8_t* memptr = &ctxt->mem[(unsigned)tmpst.mem_pos];
        if(ptr) {
            if(op->mem_len_dir <= 0)
                memcpy(ptr, memptr, copylen); // memcpy_advance()
            else
                memcpy(memptr, ptr, copylen); // memcpy_advance()
            ptr += copylen;
        }
        n += copylen;
        left -= copylen;
        tmpst.mem_len -= copylen; // have to update this so compute_????_nowrap works when op->skip>0
        tmpst.mem_pos = compute_advance(ctxt->capacity, tmpst.mem_pos, copylen); // fixup wrap around
    }

    // no padding data write for recv RX side, but we skip remaining input of packet
    const size_t padlen = compute_padlen_for_blocksize2(ctxt->granularity_pow2, n);
#ifdef TEST_WITH_FAKE_RP2040
    const size_t padlen2 = compute_padlen_for_blocksize2(ctxt->granularity == 8 ? 3 : 4, n);
    const size_t padlen3 = compute_padlen_for_blocksize(ctxt->granularity, n);
    DEBUG_ASSERT(padlen == padlen2); // GCOV_EXCL_LINE
    DEBUG_ASSERT(padlen == padlen3); // GCOV_EXCL_LINE
#endif
    // accounting update maybe packet/block based (with padding), this function API is byte based,
    //  so need to keep return value of this funciton separate from the accounting update
    size_t n_with_pad; // accounting update value
    if(op->mem_len_dir == MEM_LEN_ADDITION) {
        if(op->zero_pad) {
            // ISR FINISH can use to close packet
            uint8_t* ptr = &ctxt->mem[(unsigned)tmpst.mem_pos];
            padfill(ptr, padlen);
            tmpst.mem_pos = compute_advance(ctxt->capacity, tmpst.mem_pos, padlen);
            n_with_pad = n + padlen;
        } else {
            // don't pad 1-byte additions from ISR
            n_with_pad = n; // no padding here
        }
    } else {
        // advance padlen so we leave it aligned and pre-wraped
        tmpst.mem_pos = compute_advance(ctxt->capacity, tmpst.mem_pos, padlen);
        n_with_pad = n + padlen;
    }

    // transactionally commit new accounting
    if(op->mem_len_dir != MEM_LEN_NOCHANGE) {
        // +1 is positive and means we add (ISR driver-space uses this)
        // -1 is negative and means we subtract (user-space uses this)
        uint32_t save = spin_lock_blocking(lockid);
        DEBUG_ASSERT(assert_state(ctxt)); // GCOV_EXCL_LINE
        // mem_out update is read-modify-write written this way to try and allow compiler (the freedom)
        //  to use atomic load/store only and not generate more locking for the addition operation
        //  which is not supported by RP2040 CPU.
        // mem_len important to reload this value during read-modify-write here, as it may have
        //   changed under us (the only purpose of tmpst.mem_len was to know if the tmpst snapshot
        //   indicates an empty or full buffer) during the decision making process above
        // mem_in we don't modify here, note it also may have changed under us
        uintlen_t mem_len = state->mem_len; // implicit atomic load (into local temp) - reload
        DEBUG_ASSERT(op->mem_len_dir != 0); // GCOV_EXCL_LINE // MEM_LEN_NOCHANGE never gets here
        mem_len = (op->mem_len_dir <= 0) ? mem_len - n_with_pad : mem_len + n_with_pad;
        if(op->mem_len_dir <= 0)
            state->mem_out = tmpst.mem_pos; // implicit atomic store
        else
            state->mem_in  = tmpst.mem_pos; // implicit atomic store MEM_LEN_ADDITION
        state->mem_len = mem_len; // implicit atomic store
        DEBUG_ASSERT(assert_state(ctxt)); // GCOV_EXCL_LINE
        spin_unlock(lockid, save);
    }

    DEBUG_RETURN("xfer_buffer_recv_memcpy(%s, %s) = %ld", recv_mem_operation_to_string(op), xfer_buffer_to_string(ctxt), n);

    return n;
}

/////////////////////////////////////////////////////////
//
//  MEM
//  0......78......f0......78......f
//  ^
//  out=0 in=0                               avail=0x00 space=0x20 (need mem_len bit to know if full or empty)
//
//  ^.......^
//  out=0   in=0x8                           avail=0x08 space=0x18
//
//  ^.......................^
//  out=0                   in=0x18          avail=0x18 space=0x08
//
//          ^...............^
//          out=0x8         in=0x18          avail=0x10 space=0x10
//
//                  ^...............^
//                  out=0x10        in=0x20  avail=0x10 space=0x10
//
//  after wrapping....
//
//  ........^               ^.......
//          in=0x08         out=0x18         avail=0x10 space=0x10
//
//  ................^       ^.......
//                  in=0x10 out=0x18         avail=0x18 space=0x08
//
//
//

// spinlock is needed to transactioanlly access two or more things when a decision making
//  process in the middle, but all these things are math operations that never block.

// avail to read/load
size_t
xfer_buffer_tx_avail_raw(void)
{
    const xfer_buffer_t* const ctxt = &tx_context;
    xfer_buffer_state_t* const state = ctxt->state;
    uint32_t save = spin_lock_blocking(lockid);
    size_t res;
    // FIXME why not just res = state->mem_len ?
    //  maybe it comes from my internal friction that mem_len only needs to be 1-bit of state to
    //  record if when mem_in==mem_out that means empty or full, which makes more sense when 32bit
    if(state->mem_len < ctxt->capacity) {
        res = (state->mem_in >= state->mem_out) ? // 0..MAX-1 space
            (state->mem_in - state->mem_out) : // zero here means empty
            ((ctxt->capacity - state->mem_out) + state->mem_in);
    } else {
        res = ctxt->capacity; // full
    }
    DEBUG_RETURN("xfer_buffer_tx_avail_raw() = %zu [%s]", res, xfer_buffer_to_string(ctxt));
    DEBUG_ASSERT(res <= ctxt->capacity); // GCOV_EXCL_LINE
    spin_unlock(lockid, save);
    return res;
}

// avail to read/load (cooked view for user-space)
size_t
xfer_buffer_tx_avail(void)
{
    const xfer_buffer_t* const ctxt = &tx_context;
    size_t res = compute_offset0_blocksize(ctxt->granularity, xfer_buffer_tx_avail_raw()); // cooked view
    DEBUG_RETURN("xfer_buffer_tx_avail() = %zu [%s]", res, xfer_buffer_to_string(ctxt));
    DEBUG_ASSERT(res <= ctxt->capacity); // GCOV_EXCL_LINE
    return res;
}

// space to write/store
size_t
xfer_buffer_tx_space_raw(void)
{
    const xfer_buffer_t* const ctxt = &tx_context;
    xfer_buffer_state_t* const state = ctxt->state;
    uint32_t save = spin_lock_blocking(lockid);
    size_t res;
    if(state->mem_len > 0) {
        res = (state->mem_in >= state->mem_out) ? // 0..MAX-1 space
            (state->mem_in + state->mem_out) : // zero here means full
            (state->mem_out - state->mem_in);
    } else {
        res = ctxt->capacity; // empty (all the space)
    }
    DEBUG_RETURN("xfer_buffer_tx_space_raw() = %zu [%s]", res, xfer_buffer_to_string(ctxt));
    DEBUG_ASSERT(res <= ctxt->capacity); // GCOV_EXCL_LINE
    spin_unlock(lockid, save);
    return res;
}

// space to write/store (cooked view for user-space)
size_t
xfer_buffer_tx_space(void)
{
    const xfer_buffer_t* const ctxt = &tx_context;
    size_t res = compute_offset0_blocksize(ctxt->granularity, xfer_buffer_tx_space_raw());
    DEBUG_RETURN("xfer_buffer_tx_space() = %zu [%s]", res, xfer_buffer_to_string(ctxt));
    DEBUG_ASSERT(res <= ctxt->capacity); // GCOV_EXCL_LINE
    return res;
}

size_t
xfer_buffer_tx_capacity(void)
{
    const xfer_buffer_t* const ctxt = &tx_context;
    return ctxt->capacity;
}

size_t
xfer_buffer_tx_granularity(void)
{
    const xfer_buffer_t* const ctxt = &tx_context;
    return ctxt->granularity;
}

// avail to read/load
size_t
xfer_buffer_rx_avail_raw(void)
{
    const xfer_buffer_t* const ctxt = &rx_context;
    xfer_buffer_state_t* const state = ctxt->state;
    uint32_t save = spin_lock_blocking(lockid);
    size_t res;
    // FIXME why not just res = state->mem_len ?
    if(state->mem_len < ctxt->capacity) {
        res = (state->mem_in >= state->mem_out) ? // 0..MAX-1 avail
            (state->mem_in - state->mem_out) : // zero here means empty
            ((ctxt->capacity - state->mem_out) + state->mem_in);
    } else {
        res = ctxt->capacity; // full
    }
    DEBUG_RETURN("xfer_buffer_rx_avail_raw() = %zu [%s]", res, xfer_buffer_to_string(ctxt));
    DEBUG_ASSERT(res <= ctxt->capacity); // GCOV_EXCL_LINE
    spin_unlock(lockid, save);
    return res;
}

// avail to read/load (cooked view for user-space)
size_t
xfer_buffer_rx_avail(void)
{
    const xfer_buffer_t* const ctxt = &rx_context;
    size_t res = compute_offset0_blocksize(ctxt->granularity, xfer_buffer_rx_avail_raw());
    DEBUG_RETURN("xfer_buffer_rx_avail() = %zu [%s]", res, xfer_buffer_to_string(ctxt));
    DEBUG_ASSERT(res <= ctxt->capacity); // GCOV_EXCL_LINE
    return res;
}

// space to write/store
size_t
xfer_buffer_rx_space_raw(void)
{
    const xfer_buffer_t* const ctxt = &rx_context;
    xfer_buffer_state_t* const state = ctxt->state;
    uint32_t save = spin_lock_blocking(lockid);
    size_t res;
    if(state->mem_len > 0) {
        res = (state->mem_in >= state->mem_out) ? // 0..MAX-1 space
            (state->mem_in + state->mem_out) : // zero here means full
            (state->mem_out - state->mem_in);
    } else {
        res = ctxt->capacity; // empty (all the space)
    }
    DEBUG_RETURN("xfer_buffer_tx_space_raw() = %zu [%s]", res, xfer_buffer_to_string(ctxt));
    DEBUG_ASSERT(res <= ctxt->capacity); // GCOV_EXCL_LINE
    spin_unlock(lockid, save);
    return res;
}

// space to write/store (cooked view for user-space)
size_t
xfer_buffer_rx_space(void)
{
    const xfer_buffer_t* const ctxt = &rx_context;
    size_t res = compute_offset0_blocksize(ctxt->granularity, xfer_buffer_rx_space_raw());
    DEBUG_RETURN("xfer_buffer_rx_space() = %zu [%s]", res, xfer_buffer_to_string(ctxt));
    DEBUG_ASSERT(res <= ctxt->capacity); // GCOV_EXCL_LINE
    return res;
}

size_t
xfer_buffer_rx_capacity(void)
{
    const xfer_buffer_t* const ctxt = &rx_context;
    return ctxt->capacity;
}

size_t
xfer_buffer_rx_granularity(void)
{
    const xfer_buffer_t* const ctxt = &rx_context;
    return ctxt->granularity;
}

// Only one side is reading and another side writing.
enum BusyState /*: uint8_t*/ { // C23+ storage type
   IDLE = 0,   // No user is doing anything
   RX = 1,     // ISR has I2C TX in progress
   TX = 2,     // ISR has I2C RX in progress
   ABORT = 3   // tx_discard_all() while not IDLE
   // ABORT_RX = 4 // not yet needed
};
static atomic_char busy_state = IDLE;

static inline void
set_busy_state(enum BusyState state)
{
    DEBUG_RETURN("set_busy_state(%d)", busy_state);
    busy_state = state;  // implicit atomic store here due to C11 type
}

static inline bool
set_busy_state_ABORT_if_TX(void)
{
    // Change state to ABORT if i2c_slave_handler() is active with TX this is designed to
    // cause it to underrun until the next I2C_SLAVE_FINISH which it will reset.  This
    // prevents it trying to send garbaled output during tx_discard_all() and then the
    // CPU immediately enqueueing more data, faster than i2c_slave_handler() getting
    // next I2C_SLAVE_FINISH.
    uint8_t expected = TX;
    int res = atomic_compare_exchange_strong(&busy_state, &expected, ABORT); // TX => ABORT
    if(res) {
        // FIXME Maybe we should set timeout_abs here, to force I2C_SLAVE_FINISH to be run at timeout
        //  maybe that would also restart hardware
#if 0
        abort_timeout_abs = make_timeout_time_us(1000); // 1ms ?

        // example code, DO NOT USE HERE, move to somewhere else
        if(timeout_abs_is_lt_now(abort_timeout_abs)) { // somewhere that will be called in the future
            slvmem_i2c_deinitialize(INIT_M_DEFAULT);
            /* do reset */
            slvmem_i2c_initialize(NULL, INIT_M_DEFAULT);
        }
#endif
    }
    DEBUG_RETURN("set_busy_state_ABORT_if_TX() = %u [busy_state=%d]", res, busy_state);
    return res;
}

// Accessible to external API
int
slvmem_get_busy_state(void)
{
    // This is the only API access of this variable another CPU can use
    return (int)atomic_load(&busy_state); // implicit atomic load here due to C11 type
}

// This is used for count the length of the current i2c_slave_handler() message size
static uint8_t this_trx_len;

static void
i2c_slave_handler_receive(i2c_inst_t* i2c)
{
    const xfer_buffer_t* const ctxt = &rx_context;

    DEBUG_MESSAGE("i2c_slave_handler_receive(%p)", (void*)i2c);

    // save into memory (if there is space)
    size_t space = xfer_buffer_rx_space_raw();
    if(space == 0)  //
        return;  // FIXME we want to cause I2C NACK here

    if(busy_state == IDLE) {  // first
        // check we have space to receive a full packet
        if(space < ctxt->granularity)
            return;  // FIXME we want to cause I2C NACK condx here

        set_busy_state(RX);
        this_trx_len = 0;
    } else if(busy_state == RX) { // Nth byte
        // nominal state (check for length exceeded ?)

        // we could keep pumping data if I2C master has more, but if the specification
        // always uses fixed sized data this would only happen due to a I2C bus error
        if(this_trx_len >= ctxt->granularity) {
            set_busy_state(ABORT); // is this an error ?
            return;
        }
    } else {  // error I2C unexpected
        set_busy_state(ABORT);
        return;
    }

    // This both checks we won't block and if we can read multiple bytes
    //  (which we don't have to action all bytes, we could just read 1 byte with this method)
    size_t ravail = i2c_get_read_available(i2c);
    if(ravail > 0) {
        uint8_t tmpbuf[I2CSLAVE_BIGEST_PACKET_LEN];
        DEBUG_FILL(tmpbuf, sizeof(tmpbuf)); // poison data content to help debug

        // if the packet is zero bytes then a full packet len is remaining
        size_t packet_remaining = compute_pktrem_for_blocksize(ctxt->granularity, this_trx_len);
        //uint8_t bytes = i2c_read_bytes_raw(i2c); // 1-byte read method
        // or we could clamp this to 1 byte
        size_t clamplen = MIN(MIN(MIN(sizeof(tmpbuf), ravail), space), packet_remaining);

        // The handling policy of what to do in unexpected circumstances would be
        //  in the code just above here.  So clamplen==0 is a valid result to discard data.
        // Maybe you'd want to set set_busy_state(ABORT_RX) to discard all remaining bytes
        //  until the next I2C_HANDLER_FINISH event.
        if(clamplen > 0) {
            // FIXME investigate the I2C ~24byte FIFO and not writing one char at a time,
            //   but letting it all 16 ctxt->graunilaty bytes fill FIFO and reading that
            //   amount at a time.  This would allow us to abort if we saw a 17th byte
            //   which would be out-of-spec and indicate an error in the comms
            // NOTE maybe not app RP2040 configuration has the full 24byte FIFO ?
            i2c_read_raw_blocking(i2c, tmpbuf, clamplen); // does not block if within i2c_get_read_available()

            // this manages ringbuffer wrap around, but also it is ok if we discard
            const struct recv_mem_operation operation = {
                .data        = tmpbuf,
                .len         = clamplen,
                .skip        = 0,
                .mem_len_dir = MEM_LEN_ADDITION,
                .zero_pad    = false  // 1-byte granularity (so do not pad)
            };
            size_t copylen = xfer_buffer_recv_memcpy(&operation, ctxt);
            DEBUG_ASSERT(copylen == clamplen);
#if 1
            // The recoverage action is to discard anyway and given all the checking we did
            // this should never fail.  We add assert() above to prove that claim.
            if(copylen != clamplen) { /* do something */ }
            // section enable to remove production build -Werror compile failure
#endif
            this_trx_len += clamplen;
        }
    }
}

static void
i2c_slave_handler_request(i2c_inst_t* i2c)
{
    const xfer_buffer_t* const ctxt = &tx_context;

    DEBUG_MESSAGE("i2c_slave_handler_request(%p)", (void*)i2c);

    size_t txavail = xfer_buffer_tx_avail_raw();
    if(txavail == 0)  //
        return;  // FIXME we want to cause I2C NACK here

    if(busy_state == IDLE) {  // first
        set_busy_state(TX);

        // This load of txavail after setting IDLE => TX, is due to how tx_discard_all()
        //  and the ABORT status is managed.  Loading txavail before setting IDLE => TX
        //  may get invalidated by tx_discard_all() and we overlapped.
        txavail = xfer_buffer_tx_avail_raw(); // reload due to the way ABORT is managed
        // we don't start until at least a full packet is ready to pickup
        if(txavail < ctxt->granularity) {
            set_busy_state(IDLE); // undo state change (set-down)
            return;
        }

        this_trx_len = 0;
    } else if(busy_state == TX) { // Nth byte
        // nominal state (check for length exceeded ?)
        if(this_trx_len >= ctxt->granularity) {
            set_busy_state(ABORT); // is this an error ?  the FIFO calls for data ahead-of-time ?
            return;
        }

        txavail = xfer_buffer_tx_avail_raw();
    } else {  // error I2C unexpected
        set_busy_state(ABORT);
        return;
    }

#if 0
    // FIXME understand the design goal here
    //  surely if the I2C MASTER sents requests for data 4 times a minute the DOWNLINK will be filled
    //  with 0xff possible filling the buffer, the test eventually completes (after 2h), but the
    //  DOWNLINK has no more space to take result data.
    // If we just did nothing, the I2C TX FIFO would underrun and maybe that will cause a NACK by the
    //  RP2040 hardware otherwise the pull-up on the I2C SDA will fill with 0xff anyway ?
    // See as EIO on RP2040 I2C master.
    i2c_write_byte_raw(i2c, 0xff); // This appears to 0xff
#endif

    size_t wavail = i2c_get_write_available(i2c);
    if(wavail > 0) {
        const xfer_buffer_t* const ctxt = &tx_context;
        xfer_buffer_state_t* const state = ctxt->state;

        xfer_buffer_state_tmp_t tmpst;
        {
            uint32_t save = spin_lock_blocking(lockid);
            tmpst.mem_in  = state->mem_in;
            tmpst.mem_out = state->mem_out; // to advance this
            tmpst.mem_len = state->mem_len; // will update this (but must reload)
            spin_unlock(lockid, save);
        }
        tmpst.capacity = ctxt->capacity; // copied from const data
#ifdef TEST_WITH_FAKE_RP2040
        tmpst.mem_pos = (sizeof(tmpst.mem_pos)<<8)-1; // FIXME ???? invalidate with max value
#endif

        size_t nwavail = compute_avail_nowrap(&tmpst, tmpst.mem_out); // no wrap contiguious

        // clamp this to (I2CSLAVE_TX_PACKET_LEN - this_trx_len) so as not to overfill the TX I2C FIFO
#ifdef TEST_WITH_FAKE_RP2040
        size_t packet_offset = this_trx_len % ctxt->granularity;
        size_t packet_remaining0 = packet_offset ? ctxt->granularity - packet_offset : ctxt->granularity; // (I2CSLAVE_TX_PACKET_LEN - this_trx_len)
#endif
        // FIXME maybe need to clamp (this_trx_len < ctxt->granularity) not matter what
        size_t packet_remaining = compute_pktrem_for_blocksize2(ctxt->granularity_pow2, this_trx_len);
#ifdef TEST_WITH_FAKE_RP2040
        size_t packet_remaining2 = compute_pktrem_for_blocksize2(ctxt->granularity == 8 ? 3 : 4, this_trx_len);
        size_t packet_remaining3 = compute_pktrem_for_blocksize(ctxt->granularity, this_trx_len);
DEBUG_MESSAGE("packet_remaining %lu %lu %lu %lu  granularity=%u this_trx_len=%u", packet_remaining0, packet_remaining, packet_remaining2, packet_remaining3, ctxt->granularity, this_trx_len);
        DEBUG_ASSERT(packet_remaining == packet_remaining2); // GCOV_EXCL_LINE
        DEBUG_ASSERT(packet_remaining == packet_remaining0); // GCOV_EXCL_LINE
        DEBUG_ASSERT(packet_remaining == packet_remaining3); // GCOV_EXCL_LINE
#endif

        size_t clamplen = MIN(MIN(MIN(packet_remaining, nwavail), txavail), wavail);

        const uint8_t* src = &ctxt->mem[(unsigned)tmpst.mem_out];
        i2c_write_raw_blocking(i2c, src, clamplen); // does not block if within i2c_get_write_available()
        tmpst.mem_out = compute_advance(ctxt->capacity, tmpst.mem_out, clamplen); // fixup wrap around

        // update accounting
        {
            uint32_t save = spin_lock_blocking(lockid);
            DEBUG_ASSERT(assert_state(ctxt)); // GCOV_EXCL_LINE
            state->mem_out = tmpst.mem_out;
            uintpos_t mem_len = state->mem_len;  // implicit atomic load (compiler hint we don't need generated spinlock)
            state->mem_len = mem_len - clamplen; // read-modify-write (but we managed the spinlock)
            DEBUG_ASSERT(assert_state(ctxt)); // GCOV_EXCL_LINE
            spin_unlock(lockid, save);
        }

        this_trx_len += clamplen;
        // it is ok to just send one byte (the i2c_slave_handler() will be called again for more)
    }
}

// Note this is called by deiniitailize
static void
i2c_slave_handler_finish_internal(void)
{
    // we set check/reset alignment ready for the next packet
    if(busy_state == RX) {
        // on short RX we zero pad received data and align accounting to next packet boundary
        //  (as we don't record the exact packet length), this way user-space sees consistent zero fill
        const xfer_buffer_t* const rx_ctxt = &rx_context;
        xfer_buffer_state_t* const state = rx_ctxt->state;
        // This is a memset() to zero pad
        uintpos_t mem_in = state->mem_in; // hinting to compiler it can load this value 'once' at this pont
        const size_t padlen = compute_padlen_for_blocksize(rx_ctxt->granularity, mem_in);
        if(padlen > 0) {
            uint8_t* padptr = &rx_ctxt->mem[(unsigned)mem_in];
            // we set check/reset alignment ready for the next packet (while also padding with zero's)
            size_t padleft = padlen;
            while(padleft-- > 0)
                *padptr++ = '\0';
            DEBUG_MESSAGE("i2c_slave_handler_finish_internal() RX padlen==%lu PADDING ADDED\n", padlen);

            // always advance padlen so we leave it aligned and pre-wraped
            mem_in = compute_advance(rx_ctxt->capacity, mem_in, padlen);

            uint32_t save = spin_lock_blocking(lockid);
            DEBUG_ASSERT(assert_state(rx_ctxt)); // GCOV_EXCL_LINE
            uintlen_t mem_len = state->mem_len; // implicit atomic load (into local temp)
            state->mem_in  = mem_in;  // implicit atomic store
            mem_len += padlen; // increment local temp so no need for compiler to generate implicit spinlock
            state->mem_len = mem_len; // implicit atomic store
            DEBUG_ASSERT(assert_state_aligned_mem_in(rx_ctxt)); // GCOV_EXCL_LINE
            spin_unlock(lockid, save);
        }
    } else if(busy_state == TX || busy_state == ABORT) {
        // This should never happen, but we need to manage the I2C master not providing all the expected data
        const xfer_buffer_t* const tx_ctxt = &tx_context;
#if 1
        xfer_buffer_state_t* const state = tx_ctxt->state;
        uintpos_t mem_out = state->mem_out; // hinting to compiler it can load this value 'once' at this pont
        const size_t padlen = compute_padlen_for_blocksize(tx_ctxt->granularity, mem_out);
        if(padlen > 0) {
            // no zero pad modification, we just a skip to align for next time
            DEBUG_MESSAGE("i2c_slave_handler_finish_internal() TX padlen==%lu SKIPED\n", padlen);

            // always advance padlen so we leave it aligned and pre-wraped
            mem_out = compute_advance(tx_ctxt->capacity, mem_out, padlen);

            uint32_t save = spin_lock_blocking(lockid);
            DEBUG_ASSERT(assert_state(tx_ctxt)); // GCOV_EXCL_LINE
            uintlen_t mem_len = state->mem_len; // implicit atomic load (into local temp)
            state->mem_out  = mem_out;  // implicit atomic store
            mem_len -= padlen; // decrement local temp so no need for compiler to generate implicit spinlock
            state->mem_len = mem_len; // implicit atomic store
            DEBUG_ASSERT(assert_state_aligned_mem_out(tx_ctxt)); // GCOV_EXCL_LINE
            spin_unlock(lockid, save);
        }
#else
        // FIXME we should be able to replace the above now... :)

        // this manages ringbuffer padding and wrap around
        const struct recv_mem_operation operation = {
            .data        = NULL,
            .len         = 0, // deliberately zero to run through and add padding
            .skip        = 0,
            .mem_len_dir = MEM_LEN_ADDITION,
            .zero_pad    = true  // emit padding
        };
        size_t copylen = xfer_buffer_recv_memcpy(&operation, ctxt);
        DEBUG_ASSERT(copylen == 0);
#endif
    }
    // FIXME evaluate ABORT condition do we need separate ABORT_TX and ABORT_RX states (for correct cleanup/recovery) ?
}

static void
i2c_slave_handler_finish(i2c_inst_t *i2c)
{
    // FIXME can we ask the RP2040 i2c hardware if we saw a I2C STOP condition ?  or if this is a error cleanup due to timeout ?

    DEBUG_MESSAGE("i2c_slave_handler_finish(%p)", (void*)i2c);

    i2c_slave_handler_finish_internal(); // fixup accounting (and padding)

    // flush I2C hardware FIFO, so if for some reason there is data in FIFO we discard it
    // for some reason maybe because we decided to stop reading it as we ran out of buffer space,
    //  experienced an expected payload length, had some hardware glitch, etc...
    // if we stop reading it (from handler and let FIFO fill), it is unclear if the RP2040
    //  hardware automatically starts to NACK on the I2C SDA line, this would be ideal as it
    //  should stop the I2C master from sending.  It is unclear if that data is retried to us
    // so we attempt to reset our end to ensure the next I2C_SLAVE_RECEIVE or I2C_SLAVE_REQUEST
    //  event aligns correctly with the start of a data buffer to be processed as new packet.
    //
    // Under nominal conditions ravail==0 and nothing happens here.
    size_t ravail = i2c_get_read_available(i2c);
    while(ravail-- > 0)
        i2c_read_byte_raw(i2c); // discards from FIFO a byte at a time

    set_busy_state(IDLE);

    // used to help signal a real wakeup notify to userspace to take a look at us
    atomic_store(&i2c_notify_abs, get_absolute_time()); // 64bit write on 32bit CPU
    i2c_notify = 1; // maybe? ++ until UCHAR_MAX

#if 0
    abort_timeout_abs = 0;  // disarm this timeout
#endif
}

// Our handler is called from the I2C ISR, so it must complete quickly. Blocking calls /
// printing to stdio may interfere with interrupt handling.
static void
i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event)
{
    switch(event) {
    case I2C_SLAVE_RECEIVE: // master has written some data
        i2c_slave_handler_receive(i2c);
        break;

    case I2C_SLAVE_REQUEST: // master is requesting data
        i2c_slave_handler_request(i2c);
        break;

    case I2C_SLAVE_FINISH: // master has signalled I2C STOP / Restart
        i2c_slave_handler_finish(i2c);
        break;

    default:
        DEBUG_MESSAGE("i2c_slave_handler(%p, %d) invalid state", (void*)i2c, event);
        break;
    }
}

// Think of this like it returns the number of bytes taken
//  (but it either always takes all data offered or no data, so it's transctional in that sense)
// The transaction size is in the unit of granularity
// So this return value scheme allows it to return 3 states,
//  0 = No room to take data
//  len = Took all data
//  <= 0 Error
// The minwrite allows the caller to extend the transactional nature of taking the data
int
slvmem_tx_write_bytes(const uint8_t* data, size_t len, size_t minwrite)
{
    const xfer_buffer_t* const ctxt = &tx_context;

    DEBUG_ENTRY("slvmem_tx_write_bytes(%p, %lu, %lu)", (void*)data, len, minwrite);

    if(minwrite > ctxt->capacity)
        return INT_MIN; // invalid, can never be honored

    size_t space = xfer_buffer_tx_space_raw();
    if(space < minwrite) {
        // generates a negative error number between: -1 and -ctxt->capacity (as that is limit of minwrite)
        // this would indicate in the negative, just how short of minwrite, caller would restart after giving time to drain
        return (int)space - minwrite;
    }

    if(space < ctxt->granularity)
        return 0; // no space yet (this indicates we are currently full)

    if(len > ctxt->capacity)
        len = ctxt->capacity;	// clamp at compile time buffer limits

    // All the magic happens inside this special memcpy
    // we need to deal with ringbuffer wrap-around, ISR data visibility,
    //  both CPUs accessing accounting so a special memcpy is needed.
    const struct send_mem_operation operation = {
        .data        = data,
        .len         = len,
        .mem_len_dir = MEM_LEN_ADDITION,
        .zero_pad    = true
    };
    int res = (int) xfer_buffer_send_memcpy(&operation, ctxt);
    DEBUG_RETURN("slvmem_tx_write_bytes(%p, %lu, %lu) = %d", (void*)data, len, minwrite, res);
    return res;
}

// 'data' can be NULL, could be NULL but difficilt to think of a use-case
int
slvmem_rx_peek_bytes(uint8_t* data, size_t offpeek, size_t len, size_t minpeek)
{
    const xfer_buffer_t* const ctxt = &rx_context;

    DEBUG_ENTRY("slvmem_rx_peek_bytes(%p, offpeek=%lu, len=%lu, minpeek=%lu)", (void*)data, offpeek, len, minpeek);

    if(minpeek > ctxt->capacity)
        return INT_MIN; // invalid, can never be honored

    size_t avail = xfer_buffer_rx_avail(); // rx_avail cooked seems the correct view here
    if(avail < minpeek)
        return (int)avail - minpeek; // not enough data to meet minpeek goal

    if(avail < ctxt->granularity)
        return 0; // no data yet (this indicates we are currently empty)

    if(len > ctxt->capacity)
        len = ctxt->capacity; // clamp at compile time buffer limits

    const struct recv_mem_operation operation = {
        .data        = data,
        .len         = len,
        .skip        = offpeek,
        .mem_len_dir = MEM_LEN_NOCHANGE,
        .zero_pad    = false
    };
    int res = (int) xfer_buffer_recv_memcpy(&operation, ctxt);
    DEBUG_RETURN("slvmem_rx_peek_bytes(%p, offpeek=%lu, len=%lu, minpeek=%lu) = %d", (void*)data, offpeek, len, minpeek, res);
    return res;
}

// 'data' can be NULL, which will "cut" the data from the buffer without copy
int
slvmem_rx_read_bytes(uint8_t* data, size_t len, size_t minread)
{
    const xfer_buffer_t* const ctxt = &rx_context;

    DEBUG_ENTRY("slvmem_rx_read_bytes(%p, len=%lu, minread=%lu)", (void*)data, len, minread);

    if(minread > ctxt->capacity)
        return INT_MIN; // invalid, can never be honored

    size_t avail = xfer_buffer_rx_avail(); // rx_avail cooked seems the correct view here
    if(avail < minread)
        return (int)avail - minread; // not enough data to meet minpeek goal

    if(avail < ctxt->granularity)
        return 0; // no data yet (this indicates we are currently empty)

    if(len > ctxt->capacity)
        len = ctxt->capacity; // clamp at compile time buffer limits

    const struct recv_mem_operation operation = {
        .data        = data,
        .len         = len,
        .skip        = 0,
        .mem_len_dir = MEM_LEN_SUBTRACT,
        .zero_pad    = false
    };
    int res = (int) xfer_buffer_recv_memcpy(&operation, ctxt);
    DEBUG_RETURN("slvmem_rx_read_bytes(%p, len=%lu, minread=%lu) = %d", (void*)data, len, minread, res);
    return res;
}

// use of 'discard' term in function name should clarify to caller this API is
// destructive in nature which is the key point to get across.
void
slvmem_tx_discard_all(void)
{
    DEBUG_ENTRY("slvmem_tx_discard_all()");
    uint32_t save = spin_lock_blocking(lockid);
    // lock is acquired because we need to transactionally modify the state of
    // multiple values so if the ISR reader looks for data ready it either saw
    // the state before we locked or sees the empty state we are forcing here
    xfer_buffer_state_init(&tx_context_state);
    DEBUG_FILL(tx_context.mem, tx_context.capacity);
    // the ISR needs to be made to abort until the next I2C_SLAVE_FINISH or
    //  I2C bus STOP signal (only if it is currently in TX state)
    set_busy_state_ABORT_if_TX();
    spin_unlock(lockid, save);
    DEBUG_RETURN("slvmem_tx_discard_all()");
}

/* can_block exists to enforce the guarantee we make to the caller */
/* the API is construced like this so the user-space can write easier code to never miss a wakeup */
/*
 * the code in user-space might look like this:
 *   absolute_time_t notify_abs = slvmem_get_notify_abs();
 *   /// check conditions, like calling TX or RX or other APIs
 *   /// decide we need to wait for I2C event, and we will allow 1 millisecond
 *   absolute_time_t notify_serial = slvmem_waitfor_notify_timeout_us(notify_abs, 1000);
 *   if(notify_abs == notify_serial)
 *       /// this was a timeout
 *   else
 *       /// this was an I2C event
 *
 * Think of the notify_abs as just a 64bit serial number, it either changed or it didn't
 */
static absolute_time_t
slvmem_waitfor_notify_timeout_abs_internal(absolute_time_t notify_abs, absolute_time_t timeout_abs, bool can_block)
{
    absolute_time_t notify_abs_temp;
    size_t maxloop = can_block ? 1000 : 0; // MISRA would be proud
    do {
        uint8_t value = i2c_notify;  // implicit atomic load
        notify_abs_temp = i2c_notify_abs; // implicit atomic load

        // check condition
        if(value != 0) {
            i2c_notify = 0;  // reset
            break; // return as condition met
        }
        if(notify_abs != notify_abs_temp)
            break; // we are not allowed to sleep

        absolute_time_t now = get_absolute_time();
        if(timeout_abs_is_lt(timeout_abs, now)) {
            // paranoid expiry check (just in case best_effort_wfe_or_timeout()
            //  has quirks dependent on RP2040 config and situation)
            break;
        }

        // FIXME there is still a window here, need to:
        ///  disable_interupts()
        //   load i2c_notify_abs and i2c_notify
        //   check condition (if return, reenable_interrupt state and return)
        //   check serial condition ...
        //   check expiry (if return, reenable_interrupt state and return)
        //   wfe_restore_interupt_state()  /// does this API exist ?
        // does RP2040 support __wfe() or is it noop ? if so just use timeout timer
        //   which you can arm first, disable interrupts, recheck serial match,
        //   then decide action you want to take disarm or sleep with restore interrupts

        // FIXME check I2C handler will wakeup CPU (an ISR invocation is surely an event)
    } while(maxloop-- > 0 && !best_effort_wfe_or_timeout(timeout_abs));
    return notify_abs_temp; // timeout
}

// absolute timeout_abs (most convinent for API building compound operations with timeout)
absolute_time_t
slvmem_waitfor_notify_timeout_abs(absolute_time_t notify_abs, absolute_time_t timeout_abs)
{
    DEBUG_ENTRY("slvmem_waitfor_notify_timeout_abs(%lu, %lu)", notify_abs, timeout_abs);
    int res = slvmem_waitfor_notify_timeout_abs_internal(notify_abs, timeout_abs, true);
    DEBUG_RETURN("slvmem_waitfor_notify_timeout_abs(%lu, %lu) = %d", notify_abs, timeout_abs, res);
    return res;
}

// relative timeout_us (most convinent for API high-level callers)
absolute_time_t
slvmem_waitfor_notify_timeout_us(absolute_time_t notify_abs, uint32_t timeout_us)
{
    DEBUG_ENTRY("slvmem_waitfor_notify_timeout_us(%lu, %u)", notify_abs, timeout_us);
    const absolute_time_t timeout_abs = make_timeout_time_us(timeout_us);    // compute absolute expiry
    int res = slvmem_waitfor_notify_timeout_abs_internal(notify_abs, timeout_abs, timeout_us > 0);
    DEBUG_RETURN("slvmem_waitfor_notify_timeout_us(%lu, %u) = %d", notify_abs, timeout_us, res);
    return res;
}

// tries to wait until tx is empty
static int
slvmem_tx_drain_timeout_abs_internal(absolute_time_t timeout_abs)
{
    absolute_time_t notify_abs = slvmem_get_notify_abs();
    size_t maxloop = 1000; // MISRA would be proud
    size_t avail;
    do {
        avail = xfer_buffer_tx_avail_raw(); // tx_avail_raw aka outstanding
        if(avail == 0)
            break;  // goal met

        if(timeout_abs_is_lt_now(timeout_abs))
            break;   // timeout expired

        slvmem_waitfor_notify_timeout_abs(notify_abs, timeout_abs); // used to sleep
    } while(maxloop-- > 0);
    return avail; // non-zero means it did not complete drain
}

// this is 2nd order function built on top of other primitives, could be done in user-space but provided for convience
int
slvmem_tx_drain_timeout_abs(absolute_time_t timeout_abs)
{
    DEBUG_ENTRY("slvmem_tx_drain_timeout_abs(%lu)", timeout_abs);
    int res = slvmem_tx_drain_timeout_abs_internal(timeout_abs);
    DEBUG_RETURN("slvmem_tx_drain_timeout_abs(%lu) = %d", timeout_abs, res);
    return res;
}

// if you wanted non-blocking guarantee you'd just use xfer_buffer_tx_avail_raw() directly
// this is 2nd order function built on top of other primitives, could be done in user-space but provided for convience
int
slvmem_tx_drain_timeout_us(uint32_t timeout_us)
{
    DEBUG_ENTRY("slvmem_tx_drain_timeout_us(%u)", timeout_us);
    const absolute_time_t timeout_abs = make_timeout_time_us(timeout_us);    // compute absolute expiry
    int res = slvmem_tx_drain_timeout_abs_internal(timeout_abs);
    DEBUG_RETURN("slvmem_tx_drain_timeout_us(%u) = %d", timeout_us, res);
    return res;
}

/* flag to know if already done */
static uint8_t i2c_init_done = 0;

bool
slvmem_i2c_deinitialize(uint32_t init_mode)
{
    bool res = false;
    DEBUG_ENTRY("slvmem_i2c_deinitialize(0x%x)", init_mode);
    if(i2c_init_done || INIT_M_IS_FORCE(init_mode)) {
        i2c_init_done = 0;
        i2c_deinit(I2CSLAVE_DEVICE);

        // This seems like a sane thing to do, to force wrap up of state
        //  but also the ISR is disabled now
        i2c_slave_handler_finish_internal();

        res = true;
    }
    DEBUG_RETURN("slvmem_i2c_deinitialize(0x%x) = %d", init_mode, res);
    return res;
}

static int
slvmem_i2c_initialize_internal(const i2c_slave_config_t* cfg_values, uint32_t init_mode)
{
    static struct i2c_slave_config cfg; /* static */

    if(INIT_M_IS_INIT_LOCKS(init_mode))
        i2c_slave_lock_init();

    // we add some control here, because there is a use-case to reset the hardware
    //  but not the buffers, and retry the buffer data after reset
    if(INIT_M_IS_INIT_RX(init_mode)) {
        xfer_buffer_state_init(&rx_context_state);
        DEBUG_FILL(rx_context.mem, rx_context.capacity);
    }
    if(INIT_M_IS_INIT_TX(init_mode)) {
        xfer_buffer_state_init(&tx_context_state);
        DEBUG_FILL(tx_context.mem, tx_context.capacity);
    }

    // making this optional and storing the last configured details would allow the
    //   low-level C driver be in a position to detect an operational problem and
    //   perform a reset/restart of hardware based on previous parameters saved
    if(cfg_values)
        memcpy(&cfg, cfg_values, sizeof(cfg));

    gpio_init(cfg.sda_pin);
    gpio_set_function(cfg.sda_pin, GPIO_FUNC_I2C);
    if(cfg.use_pullups)
        gpio_pull_up(cfg.sda_pin); // FIXME check this has an effect (when function==GPIO_FUNC_I2C)

    gpio_init(cfg.scl_pin);
    gpio_set_function(cfg.scl_pin, GPIO_FUNC_I2C);
    if(cfg.use_pullups)
        gpio_pull_up(cfg.scl_pin); // FIXME check this has an effect (when function==GPIO_FUNC_I2C)

    uint res = i2c_init(I2CSLAVE_DEVICE, cfg.baudrate);
    // configure I2C hardware device for slave mode
    if(res > 0)
        i2c_slave_init(I2CSLAVE_DEVICE, cfg.address, &i2c_slave_handler);

    i2c_nack_mode = cfg.nack_mode;

    return (int) res; // baud rate set
}

int
slvmem_i2c_initialize(const i2c_slave_config_t* cfg, uint32_t init_mode)
{
    DEBUG_ENTRY("slvmem_i2c_initialize(%p %s, 0x%x)", (void*)cfg, cfg ? i2c_slave_config_to_string(cfg) : NULL, init_mode);

    int res;
    if(i2c_init_done) {
        res = INT_MIN; // special value for wrapper to generate ValueError
    } else {
        res = slvmem_i2c_initialize_internal(cfg, init_mode);

        if(res > 0)
            i2c_init_done = 1;
    }

    DEBUG_RETURN("slvmem_i2c_initialize(%p %s, 0x%x) = 0x%0x", (void*)cfg, cfg ? i2c_slave_config_to_string(cfg) : NULL, init_mode, res);
    return res;
}

//
//  This is an API to help user-space manage co-operative locking and share the TX or RX
//
//  These routines don't have to be used if the user-space code manages things so both
//  CPUs never both call a TX routine simultanously, or a RX routine simultaneously.
//
static atomic_uchar tx_userspace_lock;

// common implementation code shared
static int
slvmem_trylock_timeout_us_internal(uint32_t timeout_us, atomic_uchar* lock_address)
{
    const absolute_time_t timeout_abs = make_timeout_time_us(timeout_us);
    size_t maxloop = 1000000;
    int res = -1;
    while(maxloop-- > 0) {
        const uint8_t expected = 0;
        bool exchanged = atomic_compare_exchange_weak(lock_address, &expected, 1);
        if(exchanged) {
            res = 1;
            break;
        }
        if(timeout_us == 0) { // never blocks when zero
            res = 0;
            break;
        }
        if(timeout_abs_is_lt_now(timeout_abs)) {
            res = -1;
            break;
        }
    }
    return res;
}

int
slvmem_tx_trylock_timeout_us(uint32_t timeout_us)
{
    DEBUG_ENTRY("slvmem_tx_trylock_timeout_us(%u)", timeout_us);
    int res = slvmem_trylock_timeout_us_internal(timeout_us, &tx_userspace_lock);
    DEBUG_RETURN("slvmem_tx_trylock_timeout_us(%u) = %d", timeout_us, res);
    return res;
}

void
slvmem_tx_unlock(void)
{
    DEBUG_ENTRY("slvmem_tx_unlock()");
    atomic_store(&tx_userspace_lock, 0);
    DEBUG_RETURN("slvmem_tx_unlock()");
}

static atomic_uchar rx_userspace_lock;

int
slvmem_rx_trylock_timeout_us(uint32_t timeout_us)
{
    DEBUG_ENTRY("slvmem_rx_trylock_timeout_us(%u)", timeout_us);
    int res = slvmem_trylock_timeout_us_internal(timeout_us, &rx_userspace_lock);
    DEBUG_RETURN("slvmem_rx_trylock_timeout_us(%u) = %d", timeout_us, res);
    return res;
}

void
slvmem_rx_unlock(void)
{
    DEBUG_ENTRY("slvmem_rx_unlock()");
    atomic_store(&rx_userspace_lock, 0);
    DEBUG_RETURN("slvmem_rx_unlock()");
}
