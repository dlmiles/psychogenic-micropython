/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2025 Darryl L. Miles
 *
 */

#include <cassert>
#include <cstdarg>
#include <cstdio>

#include <fmt/format.h>

#include "fake_rp2040_api_types.h" // needed for RP2040 types
#include "test_debuglog.h"

#include "./lib/i2cslave_sock_seqpacket.h"

using namespace std;

//////////////////////////////////////////////////////////////////////////////

static char
hexnibble(uint8_t nibble)
{
    nibble &= 0x0f;
    if(nibble < 10)
        return nibble + '0';
    else
        return nibble + 'a' - 10;
}

void
hexbyte(char* buf, uint8_t b)
{
    assert(buf);
    *buf++ = hexnibble(b >> 4);
    *buf++ = hexnibble(b);
    *buf = '\0';
}

string
dump_data_to_string(void const* data, size_t len)
{
    if(len == 0)
        return "";
    string s{};
    uint8_t const* src = (uint8_t const*) data;
    size_t left = len;
    while(left-- > 0) {
        char tmpbuf[4];
        hexbyte(tmpbuf, *src++);
        s += tmpbuf;
    }
    return s;
}

string
hexbytes(void const* data, size_t len)
{
    return dump_data_to_string(data, len);
}


extern "C" {

const char*
recv_mem_operation_to_string(const struct recv_mem_operation* op)
{
    static char buf[1024];
    size_t n = snprintf(buf, sizeof(buf), ".send_mem_op={.data=%p, .len=%u, .skip=%u, .mem_len_dir=%d}", (void*)op->data, op->len, op->skip, op->mem_len_dir);
    assert(n < sizeof(buf));
    return buf;
}

const char*
send_mem_operation_to_string(const struct send_mem_operation* op)
{
    static char buf[1024];
    size_t n = snprintf(buf, sizeof(buf), ".send_mem_op={.data=%p, .len=%u, .mem_len_dir=%d, .zero_pad=%d}", (void*)op->data, op->len, op->mem_len_dir, op->zero_pad);
    assert(n < sizeof(buf));
    return buf;
}

const char*
xfer_buffer_state_to_string(const xfer_buffer_state_t* st)
{
    static char buf[1024];
    assert(st);
    size_t n = snprintf(buf, sizeof(buf), ".state={.in=%u, .out=%u, .len=%u}", st->mem_in.load(), st->mem_out.load(), st->mem_len.load());
    assert(n < sizeof(buf));
    return buf;
}

const char*
xfer_buffer_to_string(const xfer_buffer_t* xb)
{
    static char buf[1024];
    assert(xb);
    size_t n = snprintf(buf, sizeof(buf), ".xb={.mem=%p, %s, .cap=%u, .gra=%u}", (void*)xb->mem, xfer_buffer_state_to_string(xb->state), xb->capacity, xb->granularity);
    assert(n < sizeof(buf));
    return buf;
}

const char*
i2c_slave_config_to_string(const i2c_slave_config_t* cfg)
{
    static char buf[1024];
    assert(cfg);
    size_t n = snprintf(buf, sizeof(buf), ".cfg={.sda=%u, .scl=%u, .addr=%u, .baud=%u, .use_pullups=%u, .nack_mode=%u}", cfg->sda_pin, cfg->scl_pin, cfg->address, cfg->baudrate, cfg->use_pullups, cfg->nack_mode);
    assert(n < sizeof(buf));
    return buf;
}

void
DEBUG_ENTRY(const char* fmt, ...)
{
    static char buf[1024];
    va_list ap;
    va_start(ap, fmt);
    size_t n = vsnprintf(buf, sizeof(buf), fmt, ap);
    assert(n < sizeof(buf));
    va_end(ap);
    puts(buf);
}

void
DEBUG_RETURN(const char* fmt, ...)
{
    static char buf[1024];
    va_list ap;
    va_start(ap, fmt);
    size_t n = vsnprintf(buf, sizeof(buf), fmt, ap);
    assert(n < sizeof(buf));
    va_end(ap);
    puts(buf);
}

void
DEBUG_MESSAGE(const char* fmt, ...)
{
    static char buf[1024];
    va_list ap;
    va_start(ap, fmt);
    size_t n = vsnprintf(buf, sizeof(buf), fmt, ap);
    assert(n < sizeof(buf));
    va_end(ap);
    puts(buf);
}

void
DEBUG_FILL(void* p, size_t len)
{
    memset(p, 0, len);

    uint8_t* dest = (uint8_t*) p;
    uint8_t ch = rand() & 0xff;
    size_t l = len;
    while(l-- > 0)
        *dest++ = ch--;
}

bool
assert_state_aligned(const xfer_buffer_t* xb, bool aligned_mem_in, bool aligned_mem_out)
{
    const xfer_buffer_state_t* st = xb->state;

    // FIXME is this trigger enough when debugging add logging for the state

    if(st->mem_in > xb->capacity) {
        fmt::print("ASSERT(st->mem_in <= xb->capacity) {} > {}\n", st->mem_in.load(), xb->capacity);
        return false;
    }
    if(st->mem_out > xb->capacity) {
        fmt::print("ASSERT(st->mem_out <= xb->capacity) {} > {}\n", st->mem_out.load(), xb->capacity);
        return false;
    }
    if(st->mem_len > xb->capacity) {
        fmt::print("ASSERT(st->mem_len <= xb->capacity) {} > {}\n", st->mem_len.load(), xb->capacity);
        return false;
    }
    if(st->mem_len == 0 && st->mem_in != st->mem_out) {
        fmt::print("ASSERT(st->mem_len == 0 && st->mem_in == st->mem_out) {} == 0 && {} != {}\n", st->mem_len.load(), st->mem_in.load(), st->mem_out.load());
        return false;
    }

    // These are not true asserts!
    if(aligned_mem_in && st->mem_in % xb->granularity != 0) {
        fmt::print("ASSERT(st->mem_in % xb->granularity == 0) {} % {} = {} FAIL-ALIGNED-mem_in\n", st->mem_in.load(), xb->granularity, st->mem_in % xb->granularity);
        return false;
    }
    if(aligned_mem_out && st->mem_out % xb->granularity != 0) {
        fmt::print("ASSERT(st->mem_out % xb->granularity == 0) {} % {} = {} FAIL-ALIGNED-mem_out\n", st->mem_out.load(), xb->granularity, st->mem_out % xb->granularity);
        return false;
    }
#if 0
    if(st->mem_len % xb->granularity != 0) { // only a valid assert when: full or empty
        fmt::print("ASSERT(st->mem_len % xb->granularity == 0) {} % {} = {}\n", st->mem_len.load(), xb->granularity, st->mem_len % xb->granularity);
        return false;
    }
#endif

    return true;
}

bool
assert_state_aligned_mem_in(const xfer_buffer_t* xb)
{
    return assert_state_aligned(xb, true, false);
}

bool
assert_state_aligned_mem_out(const xfer_buffer_t* xb)
{
    return assert_state_aligned(xb, false, true);
}

bool
assert_state(const xfer_buffer_t* xb)
{
    return assert_state_aligned(xb, false, false);
}

} // extern "C"
