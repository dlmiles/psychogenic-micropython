/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2025 Darryl L. Miles
 *
 */

//
//
//  This is a fake version of the RP2040 C/C++ SDK
//
//  Some might call this a 'mock' API which would allow for any kind of API fault
//   injection you wanted to test.
//  You'd make these RP2040 interface API calls return unexpected conditions.
//
//  TODO skeleton I2C hardware implementation, that can unload data buffers from file,
//    or to file with a time-delay schedule.
//
//  FILE example:
//  @0000 00010203fcfdfeff
//  @0010 3031323334353637
//  @0100 414243447778797a
//
//  Where the time index is whatever we return from EPOCH from get_absolute_time().
//  Then we can inject random data, short packets, long packets, to try to break it.
//
//

#include <cassert>
#include <cstdlib>
#include <iostream>
#include <string>
#include <atomic>

#include <time.h>
#include <sys/time.h>
#include <sys/select.h>

#include <fmt/base.h>
#include <fmt/format.h>

#include "fake_rp2040_api.h"
#include "test_debuglog.h"   // hexbytes

using namespace std;

//////////////////////////////////////////////////////////////////////////////

int error_count = 0;

static void
assert_if(bool cond, string const& msg)
{
    if(!cond) {
        fmt::print("ASSERT: {}\n", msg);
        error_count++;
    }
}


// C++ helper stuff not RP2040 API
static constexpr unsigned int USEC_PER_SECOND = 1000000;

static struct timeval
tv_now(void)
{
    struct timeval now;
    gettimeofday(&now, nullptr);
    return now;
}

static absolute_time_t
tv_to_absolute_time(struct timeval const& tv)
{
    return ((int64_t)tv.tv_sec * USEC_PER_SECOND) + tv.tv_usec;
}

static struct timeval
absolute_time_to_tv(absolute_time_t us)
{
    assert(us <= INT64_MAX);
    struct timeval tv = { .tv_sec = (int64_t)(us / USEC_PER_SECOND), .tv_usec = (int64_t)(us % USEC_PER_SECOND) };
    return tv;
}

static string
p(void const* p)
{
    char tmpbuf[32];
    auto n = snprintf(tmpbuf, sizeof(tmpbuf), "%p", p);
    assert((ssize_t)n < (ssize_t)sizeof(tmpbuf));
    return tmpbuf;
}

static atomic<int64_t> epoch;
// EPOCH is the lowest get_absolute_time() we issued, like the startup time
//  of the testing, this is used for the diagnoatic output so it does not
//  contain large numbers difficult to reason about.
// The syntax "us1234" is used to mean 'time index 1234 micro-seconds since EPOCH'
//  which is much easier to reason about from the logs.

static absolute_time_t
check_set_epoch(void)
{
    auto now = tv_to_absolute_time(tv_now());
    if(epoch)
        return now;
    fmt::print("EPOCH = {} = us{}\n", now, 0);
    epoch = now;
    return now;
}

static string
tm(absolute_time_t const& t)
{
    auto now = check_set_epoch();
    if((int64_t)t < epoch.load() || now == 0)
        return fmt::format("{}", now); // less than now
    return fmt::format("us{}", t - epoch);
}

// C++ stuff
static absolute_time_t
compute_timeout_remaining(absolute_time_t abs)
{
    auto now = get_absolute_time();
    auto res = (abs > now) ? abs - now : 0;
    fmt::print("compute_timeout_remaining(abs={}) = {} [now={}]\n", tm(abs), res, tm(now));
    return res;
}

// C stuff (matching RP2040 API)
extern "C" {

i2c_inst_t i2c1_inst;

static constexpr size_t SPIN_LOCKS_COUNT = 32;
spin_lock_t SPIN_LOCKS[SPIN_LOCKS_COUNT];

bool
best_effort_wfe_or_timeout(absolute_time_t timeout_timestamp)
{
    bool res = true; // could be an event or timeout
    while(1) {
        absolute_time_t remaining = compute_timeout_remaining(timeout_timestamp);
        if(remaining <= 0)
            break; // timeout
        struct timeval tv_remaining = absolute_time_to_tv(remaining);
        assert(tv_remaining.tv_sec >= 0);
        assert(tv_remaining.tv_usec >= 0);
        // using select for usec wall-clock delay
        int n = select(0, nullptr, nullptr, nullptr, &tv_remaining);
        assert(n >= 0);
        if(n == 0)
            break;
    }
    fmt::print("best_effort_wfe_or_timeout(timeout_timestamp={}) = {}\n", tm(timeout_timestamp), res);
    return res;
}

uint64_t
time_us_64(void)
{
    auto res = get_absolute_time();  // FIXME check this is correct
    fmt::print("time_us_64() = {}\n", res);
    return res;
}

absolute_time_t
get_absolute_time(void)
{
    auto now = check_set_epoch();
    fmt::print("get_absolute_time() = {} [{}]\n", now, tm(now));
    return now;
}

absolute_time_t
make_timeout_time_us(uint64_t us)
{
    auto now = get_absolute_time();
    auto res = now + us;
    fmt::print("make_timeout_time_us(us={}) = {} [now={}]\n", us, tm(res), tm(now));
    return res;
}

uint
next_striped_spin_lock_num(void)
{
    uint res = 24;
    // FIXME inject fault for coverage
    fmt::print("next_striped_spin_lock_num() = {}\n", res);
    return res;
}

int
spin_lock_claim_unused(bool required)
{
    // FIXME inject fault for coverage
    if(required) {
        fmt::print("PANIC: required={}\n", required);
        assert(false);
    }
    int res = -1; // error
    size_t n = 0;
    for(auto& it : SPIN_LOCKS) {
        if(!it.claimed) {
            it.claimed = true;
            res = n;
            break;
        }
        n++;
    }
    fmt::print("spin_lock_claim_unused(required={}) = {}\n", required, res);
    return res; // error
}

spin_lock_t*
spin_lock_init(uint lock_num)
{
    fmt::print("spin_lock_init(lock_num={})\n", lock_num);

    assert(lock_num < SPIN_LOCKS_COUNT);

    auto lock = &SPIN_LOCKS[lock_num];
    auto res = pthread_spin_init(&lock->lock, PTHREAD_PROCESS_PRIVATE);
    assert(res == 0);
    lock->value = 1;
    return lock;
}

uint32_t
spin_lock_blocking(spin_lock_t* lock)
{
    auto r = rand();
    fmt::print("spin_lock(lock={}) = saved_irq={}\n", p(lock), r);

    assert(lock);

    auto res = pthread_spin_trylock(&lock->lock);
    if(res != 0)
        fmt::print("ERROR: pthread_spin_trylock() = {} errno={} {} FAILED\n", res, errno, res == EDEADLOCK ? "EDEADLOCK" : "EBUSY");
    assert(res == 0);
    lock->saved_irq = r;
    lock->value++;
    return lock->saved_irq;
}

void
spin_unlock(spin_lock_t* lock, uint32_t saved_irq)
{
    fmt::print("spin_unlock(lock={}, saved_irq={})\n", p(lock), saved_irq);

    assert(lock);
    assert(saved_irq == lock->saved_irq);

    auto res = pthread_spin_unlock(&lock->lock);
    assert(res == 0);
}

void
gpio_init(uint gpio)
{
    fmt::print("gpio_init(gpio={})\n", gpio);
}

void
gpio_set_function(uint gpio, gpio_function_t fn)
{
    fmt::print("gpio_set_function(gpio={}, fn={})\n", gpio, (int)fn);
}

void
gpio_set_pulls(uint gpio, bool up, bool down)
{
    fmt::print("gpio_set_pulls(gpio={}, up={}, down={})\n", gpio, up, down);
}

void
gpio_pull_up(uint gpio)
{
    fmt::print("gpio_pull_up(gpio={})\n", gpio);
}

uint
i2c_init(i2c_inst_t* i2c, uint baudrate)
{
    fmt::print("i2c_init(i2c={}, baudrate={}) = {}\n", p(i2c), baudrate, baudrate);

    assert(i2c);
    assert(baudrate > 0);

    i2c->has_inited = true;
    return baudrate;
}

void
i2c_deinit(i2c_inst_t* i2c)
{
    fmt::print("i2c_deinit(i2c={})\n", p(i2c));

    assert(i2c);
    assert_if(i2c->has_inited, fmt::format("i2c->has_inited == {}", i2c->has_inited)); // maybe this is allowed

    i2c->has_inited = false;
}


// original has this inlined
size_t
i2c_get_read_available(i2c_inst_t* i2c)
{
    assert(i2c);

    size_t res = i2c->rx_avail;

    fmt::print("i2c_get_read_available(i2c={}) = {}\n", p(i2c), res);

    return res;
}

// original has this inlined
size_t
i2c_get_write_available(i2c_inst_t* i2c)
{
    assert(i2c);

    size_t res = i2c->tx_space;

    fmt::print("i2c_get_write_available(i2c={}) = {}\n", p(i2c), res);

    return res;
}

static i2c_slave_handler_t i2c_slave_handler_func;

void
i2c_slave_init(i2c_inst_t* i2c, uint8_t address, i2c_slave_handler_t handler)
{
    fmt::print("i2c_slave_init(i2c={}, address={:x}, handler={})\n", p(i2c), address, p((void const*)handler));

    assert(i2c);
    assert(address >= 1 && address <= 127);
    assert(handler);
    assert(i2c->has_inited);  // check i2c_init() was called first

    i2c->is_slave = true;


    i2c->rx_cap   = sizeof(i2c->rxbuf);
    i2c->rx_avail = 0; // empty
    i2c->rx_pos   = 0;
    memset(i2c->rxbuf, 0, sizeof(i2c->rxbuf));

    i2c->tx_cap   = sizeof(i2c->txbuf);
    i2c->tx_space = 0; // full
    i2c->tx_pos   = 0;
    memset(i2c->txbuf, 0, sizeof(i2c->txbuf));

    i2c_slave_handler_func = handler;
}

void
i2c_slave_deinit(i2c_inst_t* i2c)
{
    fmt::print("i2c_slave_deinit(i2c={})\n", p(i2c));

    assert(i2c);
    assert(i2c->is_slave);

    i2c->is_slave = false;
}

void
i2c_read_raw_blocking(i2c_inst_t* i2c, uint8_t* dst, size_t len)
{
    assert(i2c);
    assert(dst);

    if(i2c->rx_avail == 0) { // called with no data // blocking ?
        fmt::print("i2c_read_raw_blocking(i2c={}, dst={}, len={}) = void  ERROR rx_avail={} NODATA as EMPTY\n", p(i2c), p(dst), len, i2c->rx_avail);
        return;
    }

    size_t rlen = std::min(len, i2c->rx_avail);
    if(rlen > 0) {
        assert(i2c->rx_pos + rlen < sizeof(i2c->rxbuf));
        uint8_t const* src = &i2c->rxbuf[i2c->rx_pos];
        memcpy(dst, src, rlen);
        i2c->rx_pos   += rlen;
        i2c->rx_avail -= rlen;
    }

    fmt::print("i2c_read_raw_blocking(i2c={}, dst={}, len={}) = [{}]\n", p(i2c), p(dst), len, dump_data_to_string(dst, rlen));
}

uint8_t
i2c_read_byte_raw(i2c_inst_t* i2c)
{
    assert(i2c);

    if(i2c->rx_avail == 0) { // called with no data // blocking ?
        fmt::print("i2c_read_byte_raw(i2c={}) = {}  ERROR rx_avail={} NODATA as EMPTY\n", p(i2c), 0, i2c->rx_avail);
        return 0;
    }

    uint8_t res = i2c->rxbuf[i2c->rx_pos];
    i2c->rx_pos++;
    i2c->rx_avail--;

    fmt::print("i2c_read_byte_raw(i2c={}) = {}\n", p(i2c), res);

    return res;
}

void
i2c_write_raw_blocking(i2c_inst_t* i2c, const uint8_t* src, size_t len)
{
    assert(i2c);
    assert(src);

    if(i2c->tx_space == 0) {
        fmt::print("i2c_read_byte_raw(i2c={}) = void  ERROR tx_space={} NOROOM as FULL\n", p(i2c), i2c->tx_space);
        return;
    }

    size_t wlen = std::min(len, i2c->tx_space);
    if(wlen > 0) {
        assert(i2c->tx_pos + wlen < sizeof(i2c->txbuf));
        uint8_t* dest = &i2c->txbuf[i2c->tx_pos];
        memcpy(dest, src, wlen);
        i2c->tx_pos   += wlen;
        i2c->tx_space -= wlen;
    }

    fmt::print("i2c_write_raw_blocking(i2c={}, src={}, len={}) = [{}]\n", p(i2c), p(src), len, dump_data_to_string(src, wlen));
}

} // extern "C"


//////////////////////////////////////////////////////////////////////////////

static void
invoke_i2c_slave_handler(i2c_inst_t* i2c, i2c_slave_event_t event)
{
    assert(i2c_slave_handler_func); // handler not set ?
    (*i2c_slave_handler_func)(i2c, event);
}

int
kick_i2c_receive_handler(size_t avail, size_t waitfor, uint8_t const* data/*= nullptr*/, size_t len/* = 0*/)
{
    assert(i2c1_inst.rx_avail == 0); // FIXME need to compact()
    if(1) { // compact
        memmove(i2c1_inst.rxbuf, &i2c1_inst.rxbuf[i2c1_inst.rx_pos], i2c1_inst.rx_avail);
        i2c1_inst.rx_pos = 0;
    }
    assert(i2c1_inst.rx_pos + i2c1_inst.rx_avail + len < sizeof(i2c1_inst.rxbuf));
    if(data)
        memcpy(&i2c1_inst.rxbuf[i2c1_inst.rx_pos], data, len);

    size_t pos = i2c1_inst.rx_pos; // save for log output
    i2c1_inst.rx_avail += avail; // you probably want avail==len
    assert(i2c1_inst.rx_avail == avail); // for now we did not test more complex state

    size_t seen = 0;
    do {
        invoke_i2c_slave_handler(&i2c1_inst, I2C_SLAVE_RECEIVE);
        // check API call and report seen
        size_t newseen = avail - i2c1_inst.rx_avail;
        if(seen == newseen) {  // nothing was added
            fmt::print("I2C_SLAVE_RECEIVE nothing was added, avail={} waitfor={} seen={} EARLY EXIT, use I2C_SLAVE_FINISH\n", avail, waitfor, seen);
            break;
        }
        seen = newseen;
    } while(waitfor > seen);

    // seen contains the total bytes seen (taken by driver ISR)
    fmt::print("kick_i2c_receive_handler(avail={}, waitfor={}) = [{}] rx_avail={}\n", avail, waitfor, hexbytes(&i2c1_inst.rxbuf[pos], seen), i2c1_inst.rx_avail);
    i2c1_inst.rx_avail = 0; // FIXME remove this ?

    return 0;
}

int
kick_i2c_request_handler(size_t space, size_t waitfor)
{
    assert(i2c1_inst.tx_space == 0); // FIXME what to do ?
    assert(i2c1_inst.tx_pos + i2c1_inst.tx_space + space < sizeof(i2c1_inst.txbuf));
    size_t pos = i2c1_inst.tx_pos; // save for log output
    i2c1_inst.tx_space += space;
    size_t seen = 0;
    do {
        invoke_i2c_slave_handler(&i2c1_inst, I2C_SLAVE_REQUEST);
        // check API call and report seen
        size_t newseen = space - i2c1_inst.tx_space;
        if(seen == newseen) {  // nothing was added
            fmt::print("I2C_SLAVE_REQUEST nothing was added, TX UNDERRUN, space={} waitfor={} seen={} EARLY EXIT, use I2C_SLAVE_FINISH\n", space, waitfor, seen);
            break;
        }
        seen = newseen;
    } while(waitfor > seen);

    // seen contains the total bytes seen (provided by driver ISR)
    fmt::print("kick_i2c_request_handler(space={}, waitfor={}) = [{}] tx_space={}\n", space, waitfor, hexbytes(&i2c1_inst.txbuf[pos], seen), i2c1_inst.tx_space);

    i2c1_inst.tx_pos = 0; // reset

    i2c1_inst.tx_space = 0; // FIXME remove this ?

    return 0;
}

int
kick_i2c_finish_handler(void)
{
    fmt::print("kick_i2c_finish_handler() entry");
    invoke_i2c_slave_handler(&i2c1_inst, I2C_SLAVE_FINISH);
    // check API call and report seen
    fmt::print("kick_i2c_finish_handler() exit");
    return 0;
}

void
kick_i2c_bogus_handler(int bogus_event)
{
    invoke_i2c_slave_handler(&i2c1_inst, (i2c_slave_event_t)bogus_event);
}

void
fake_rp2040_reset(void)
{
    // FIXME reset the state per test, memset() all the globals in here ?
}
