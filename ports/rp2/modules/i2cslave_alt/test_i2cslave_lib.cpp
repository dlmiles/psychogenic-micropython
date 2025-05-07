
#include <gtest/gtest.h>

#include <fmt/format.h>

#include "fake_rp2040_api.h"	// absolute_time_t
#include "test_debuglog.h"
#include "./lib/i2cslave_sock_seqpacket.h"

#include <stdlib.h>
#include <time.h>

using namespace std;

/////////////////////////////////////////////////////////////////////////////

// test

void breakpoint(void)
{
   fmt::print("BREAKPOINT\n");
}

static void
tx_report(void)
{
    auto avail_raw = xfer_buffer_tx_avail_raw();
    fmt::print("xfer_buffer_tx_avail_raw() = {}\n", avail_raw);

    auto space_raw = xfer_buffer_tx_space_raw();
    fmt::print("xfer_buffer_tx_space_raw() = {}\n", space_raw);

    auto avail = xfer_buffer_tx_avail();
    fmt::print("xfer_buffer_tx_avail() = {}\n", avail);

    auto space = xfer_buffer_tx_space();
    fmt::print("xfer_buffer_tx_space() = {}\n", space);

    auto capacity = xfer_buffer_tx_capacity();
    fmt::print("xfer_buffer_tx_capacity() = {}\n", capacity);
    assert(capacity > 0);

    auto granularity = xfer_buffer_tx_granularity();
    fmt::print("xfer_buffer_tx_granularity() = {}\n", granularity);
    assert(granularity > 0);

    assert(avail <= capacity);
    assert(space <= capacity);
    assert(granularity <= capacity);
    // FIXME  check range limits of _raw to cooked
    //assert(avail_raw < avail);
    assert(avail_raw + space_raw == capacity);
    assert(avail + space == capacity);
}

static void
rx_report(void)
{
    auto avail_raw = xfer_buffer_rx_avail_raw();
    fmt::print("xfer_buffer_rx_avail_raw() = {}\n", avail_raw);

    auto space_raw = xfer_buffer_rx_space_raw();
    fmt::print("xfer_buffer_rx_space_raw() = {}\n", space_raw);

    auto avail = xfer_buffer_rx_avail();
    fmt::print("xfer_buffer_rx_avail() = {}\n", avail);

    auto space = xfer_buffer_rx_space();
    fmt::print("xfer_buffer_rx_space() = {}\n", space);

    auto capacity = xfer_buffer_rx_capacity();
    fmt::print("xfer_buffer_rx_capacity() = {}\n", capacity);
    assert(capacity > 0);

    auto granularity = xfer_buffer_rx_granularity();
    fmt::print("xfer_buffer_rx_granularity() = {}\n", granularity);
    assert(granularity > 0);

    assert(avail <= capacity);
    assert(space <= capacity);
    assert(granularity <= capacity);
    assert(avail_raw + space_raw == capacity);
    assert(avail + space == capacity);
}

static void
report(string const& s, int trx = 0)
{
   fmt::print("#### REPORT: {}\n", s);
   if(trx < 0)
       rx_report();
   else if(trx > 0)
       tx_report();
   fmt::print("###\n");
}

static bool
my_test001(void)
{
    fake_rp2040_reset();

    auto start = get_absolute_time(); // also sets EPOCH

    struct i2c_slave_config const cfg_values = {
       .sda_pin = 2,
       .scl_pin = 3,
       .address = 0x51, // see example.py
       .baudrate = 100000,
       .use_pullups = 0,
       .nack_mode = NACK_M_NONE
    };
    uint32_t init_mode = INIT_M_DEFAULT;
    auto r0 = slvmem_i2c_initialize(&cfg_values, init_mode);
    assert(r0 == 100000);
    auto r1 = slvmem_i2c_initialize(&cfg_values, init_mode);
    assert(r1 == INT_MIN);

    rx_report();
    tx_report();

    fmt::print("MARK01\n");
    auto txd1 = slvmem_tx_drain_timeout_us(0);
    assert(txd1 == 0);
    auto txd2 = slvmem_tx_drain_timeout_us(0);
    assert(txd2 == 0);
    auto txd3 = slvmem_tx_drain_timeout_abs(get_absolute_time());
    assert(txd3 == 0);
    auto txd4 = slvmem_tx_drain_timeout_abs(get_absolute_time() + 1000); // +1ms
    assert(txd4 == 0);

    auto bs = slvmem_get_busy_state();
    assert(bs == 0);

    fmt::print("MARK02\n");
    auto notify_abs = slvmem_get_notify_abs();
    auto notify1 = slvmem_waitfor_notify_timeout_us(notify_abs, 0);
    assert(notify1 == notify_abs);
    fmt::print("MARK03\n");
    auto notify2 = slvmem_waitfor_notify_timeout_us(notify_abs, 1);
    assert(notify2 == notify_abs);
    fmt::print("MARK04\n");
    auto notify3 = slvmem_waitfor_notify_timeout_abs(notify_abs, get_absolute_time());
    assert(notify3 == notify_abs);
    fmt::print("MARK05\n");
    auto notify4 = slvmem_waitfor_notify_timeout_abs(notify_abs, get_absolute_time() + 1000); // +1ms
    assert(notify4 == notify_abs);

    auto notify_abs_invalid = notify_abs - 1; // invalidate it
    notify1 = slvmem_waitfor_notify_timeout_us(notify_abs_invalid, 0);
    assert(notify1 == notify_abs);
    notify2 = slvmem_waitfor_notify_timeout_us(notify_abs_invalid, 1);
    assert(notify2 == notify_abs);
    notify3 = slvmem_waitfor_notify_timeout_abs(notify_abs_invalid, get_absolute_time());
    assert(notify3 == notify_abs);
    notify4 = slvmem_waitfor_notify_timeout_abs(notify_abs_invalid, get_absolute_time() + 1000); // +1ms
    assert(notify4 == notify_abs);

    fmt::print("MARK06\n");
    slvmem_tx_discard_all();

    auto rxlck = slvmem_rx_trylock_timeout_us(0); // wins
    assert(rxlck == 1);
    rxlck = slvmem_rx_trylock_timeout_us(0); // fail
    assert(rxlck == 0);
    rxlck = slvmem_rx_trylock_timeout_us(1); // fail
    assert(rxlck < 0); // timeout

    fmt::print("MARK07\n");
    uint8_t pkbuf[4096];
    auto pk = slvmem_rx_peek_bytes(pkbuf, 0, 0, 0);
    assert(pk == 0);
    pk = slvmem_rx_peek_bytes(nullptr, 0, 0, 0);
    assert(pk == 0);
    pk = slvmem_rx_peek_bytes(pkbuf, 1, 0, 0);
    assert(pk == 0);
    pk = slvmem_rx_peek_bytes(nullptr, 1, 0, 0);
    assert(pk == 0);
    pk = slvmem_rx_peek_bytes(nullptr, 0, 0, INT_MAX); // invalid request
    assert(pk == INT_MIN);
    pk = slvmem_rx_peek_bytes(nullptr, 0, 0, xfer_buffer_rx_capacity());
    const int neg_tx_capacity = -(int)xfer_buffer_rx_capacity(); // negate
    assert(pk == neg_tx_capacity);
    pk = slvmem_rx_peek_bytes(nullptr, 0, 0, 1);
    assert(pk == -1);

    fmt::print("MARK08\n");
    uint8_t rxbuf[4096];
    auto rx = slvmem_rx_read_bytes(rxbuf, 0, 0);
    assert(rx == 0);
    rx = slvmem_rx_read_bytes(nullptr, 0, 0);
    assert(rx == 0);
    rx = slvmem_rx_read_bytes(rxbuf, 1, 0);
    assert(rx == 0);
    rx = slvmem_rx_read_bytes(nullptr, 1, 0);
    assert(rx == 0);

    slvmem_rx_unlock();
    rxlck = slvmem_rx_trylock_timeout_us(0);
    assert(rxlck == 1); // lock acquired
    slvmem_rx_unlock();

    auto txlck = slvmem_tx_trylock_timeout_us(0); // wins
    assert(txlck == 1);
    txlck = slvmem_tx_trylock_timeout_us(0); // fail
    assert(txlck == 0);
    txlck = slvmem_tx_trylock_timeout_us(1); // fail
    assert(txlck < 0); // timeout

    fmt::print("MARK09\n");
    uint8_t txbuf[4096];
    auto tx = slvmem_tx_write_bytes(txbuf, 0, 0);
    assert(tx == 0);
    tx = slvmem_tx_write_bytes(nullptr, 0, 0);
    assert(tx == 0);
    tx = slvmem_tx_write_bytes(txbuf, 0, xfer_buffer_tx_capacity() + 1);
    assert(tx == INT_MIN); // request can never succeed
    tx = slvmem_tx_write_bytes(txbuf, 0, INT_MAX);
    assert(tx == INT_MIN); // request can never succeed

    slvmem_tx_unlock();
    txlck = slvmem_tx_trylock_timeout_us(0);
    assert(txlck == 1); // lock acquired
    slvmem_tx_unlock();

    fmt::print("MARK10\n");
    auto txd5 = slvmem_tx_drain_timeout_us(0);
    assert(txd5 == 0);
    auto txd6 = slvmem_tx_drain_timeout_abs(get_absolute_time());
    assert(txd6 == 0);
    auto txd7 = slvmem_tx_drain_timeout_abs(get_absolute_time() + 1000); // +1ms
    assert(txd7 == 0);

    fmt::print("MARK11\n");
    notify_abs = slvmem_get_notify_abs();
    auto notify5 = slvmem_waitfor_notify_timeout_us(notify_abs, 0);
    assert(notify5 == notify_abs);
    fmt::print("MARK12\n");
    auto notify6 = slvmem_waitfor_notify_timeout_us(notify_abs, 1);
    assert(notify6 == notify_abs);
    fmt::print("MARK13\n");
    auto notify7 = slvmem_waitfor_notify_timeout_abs(notify_abs, get_absolute_time());
    assert(notify7 == notify_abs);
    fmt::print("MARK14\n");
    auto notify8 = slvmem_waitfor_notify_timeout_abs(notify_abs, get_absolute_time() + 1000); // +1ms
    assert(notify8 == notify_abs);

    fmt::print("MARK14\n");
    slvmem_tx_discard_all();

    rx_report();
    tx_report();

    auto de1 = slvmem_i2c_deinitialize(init_mode);
    assert(de1);
    auto de2 = slvmem_i2c_deinitialize(init_mode);
    assert(!de2);
    auto de3 = slvmem_i2c_deinitialize(INIT_M_FORCE);
    assert(de3);

    fmt::print("#### TEST my_test001() = {}\n", true);

    return true;
}

bool
my_test002(void)
{
    fake_rp2040_reset();

    auto start = get_absolute_time(); // also sets EPOCH

    struct i2c_slave_config const cfg_values = {
       .sda_pin = 2,
       .scl_pin = 3,
       .address = 0x51, // see example.py
       .baudrate = 100000,
       .use_pullups = 0,
       .nack_mode = NACK_M_NONE
    };
    uint32_t init_mode = INIT_M_DEFAULT;
    auto r0 = slvmem_i2c_initialize(&cfg_values, init_mode);
    assert(r0 == 100000);
    auto r1 = slvmem_i2c_initialize(&cfg_values, init_mode);
    assert(r1 == INT_MIN);

    rx_report();
    tx_report();

    //////////////////////////////////////////////////////////////////////////////////

    auto txlck = slvmem_tx_trylock_timeout_us(0); // wins
    assert(txlck == 1);

    fmt::print("MARK09\n");
    uint8_t txbuf[4096];
    memset(txbuf, 0x5a, sizeof(txbuf));
    size_t len = xfer_buffer_tx_capacity();
    auto tx = slvmem_tx_write_bytes(txbuf, 0, len);
    assert(tx == 0);
    tx = slvmem_tx_write_bytes(txbuf, len, len);
    assert(tx == (int)len); // success

    report("slvmem_tx_write_bytes()", 1);

    slvmem_tx_unlock();

    auto txd5 = slvmem_tx_drain_timeout_us(0);
    assert(txd5 == (int)len); // drain never happens
    auto txd6 = slvmem_tx_drain_timeout_abs(get_absolute_time());
    assert(txd6 == (int)len);
    auto txd7 = slvmem_tx_drain_timeout_abs(get_absolute_time() + 1000); // +1ms
    assert(txd7 == (int)len);

    auto space = xfer_buffer_tx_space();
    assert(space == 0);
    auto avail = xfer_buffer_tx_avail();
    assert(avail == len);

    space = xfer_buffer_tx_space_raw();
    assert(space == 0);
    avail = xfer_buffer_tx_avail_raw();
    assert(avail == len);

    // FIXME simulate I2C hardware callbacks (check tx is still blocked until it can fit a whole packet

    kick_i2c_request_handler(1, 0); // put all data in RX FIFO (real RP2040 only has 24byte FIFO)
    tx = slvmem_tx_write_bytes(txbuf, 1, 0);
    assert(tx == 0);

    kick_i2c_request_handler(2, 0); // put all data in RX FIFO (real RP2040 only has 24byte FIFO)
    tx = slvmem_tx_write_bytes(txbuf, 1, 0);
    assert(tx == 0);

    kick_i2c_request_handler(16 - 3, 0); // put all data in RX FIFO (real RP2040 only has 24byte FIFO)
    tx = slvmem_tx_write_bytes(txbuf, 1, 0);
    assert(tx == 1);  // now it takes it (expecting to see it padded with 15xNUL bytes)

    kick_i2c_finish_handler();

    tx = slvmem_tx_write_bytes(txbuf, 1, 0);
    assert(tx == 0);  // blocked again

    auto txd = slvmem_tx_drain_timeout_us(0);
    assert((size_t)txd == len);
    assert((size_t)txd == xfer_buffer_tx_capacity());

    // BIG MESSAGE
    //  FIXME check it took one packet and discarded the rest
    kick_i2c_request_handler(xfer_buffer_tx_capacity(), 0); // put all data in RX FIFO (real RP2040 only has 24byte FIFO)
    tx = slvmem_tx_write_bytes(txbuf, 2, 0);
    assert(tx == 2);  // now it takes it (expecting to see it padded with 14xNUL bytes)

    tx = slvmem_tx_write_bytes(txbuf, 1, 0);
    assert(tx == 0);  // blocked again

    kick_i2c_finish_handler();

    txd = slvmem_tx_drain_timeout_us(0);
    assert(txd == (int)len); // drain never happens
    txd = slvmem_tx_drain_timeout_abs(get_absolute_time());
    assert(txd == (int)len);
    txd = slvmem_tx_drain_timeout_abs(get_absolute_time() + 1000); // +1ms
    assert(txd == (int)len);

    kick_i2c_request_handler(xfer_buffer_tx_capacity(), 0); // put all data in RX FIFO (real RP2040 only has 24byte FIFO)

    kick_i2c_finish_handler();

    /////////////////////////////////////////////////////////////////////////////////

    uint8_t pkbuf[4096];
    uint8_t rxbuf[4096];

    kick_i2c_receive_handler(1, 0);
    auto pk = slvmem_rx_peek_bytes(pkbuf, 0, 1, 0);
    assert(pk == 0);
    auto rx = slvmem_rx_read_bytes(rxbuf, 1, 0);
    assert(rx == 0);

    kick_i2c_receive_handler(2, 0);
    pk = slvmem_rx_peek_bytes(pkbuf, 0, 1, 0);
    assert(pk == 0);
    rx = slvmem_rx_read_bytes(rxbuf, 1, 0);
    assert(rx == 0);

    kick_i2c_receive_handler(4, 0);
    pk = slvmem_rx_peek_bytes(pkbuf, 0, 1, 0);
    assert(pk == 0);
    rx = slvmem_rx_read_bytes(rxbuf, 1, 0);
    assert(rx == 0);

    kick_i2c_receive_handler(1, 0); // FUZZ more data than expected here
    pk = slvmem_rx_peek_bytes(pkbuf, 0, 1, 0);
    assert(pk == 0);
    rx = slvmem_rx_read_bytes(rxbuf, 1, 0);
    assert(rx == 0);

    kick_i2c_finish_handler();

    pk = slvmem_rx_peek_bytes(nullptr, 0, 0, 9);
    assert(pk == -1); // 1 short
    pk = slvmem_rx_peek_bytes(nullptr, 0, 0, 17);
    assert(pk == -9); // 9 short
    assert(sizeof(pkbuf) > xfer_buffer_rx_capacity());
//    pk = slvmem_rx_peek_bytes(pkbuf, 0, sizeof(pkbuf), 0); // bigger than capacity
fmt::print("pk={}\n", pk);
//    assert(pk == 16); // HUH
    pk = slvmem_rx_peek_bytes(pkbuf, 0, 8, 0);
    assert(pk == 8);
breakpoint();
    pk = slvmem_rx_peek_bytes(pkbuf, 6, 8, 0);
fmt::print("pk={} avail={} {}\n", pk, xfer_buffer_rx_avail(), xfer_buffer_rx_avail_raw());
    assert(pk == 2);
    pk = slvmem_rx_peek_bytes(pkbuf, 8, 8, 0);
    assert(pk == 0);
    pk = slvmem_rx_peek_bytes(pkbuf, sizeof(pkbuf), 8, 0);
    assert(pk == 0);
    pk = slvmem_rx_peek_bytes(pkbuf, INT_MAX, 8, 0);
    assert(pk == 0);
    pk = slvmem_rx_peek_bytes(pkbuf, sizeof(pkbuf), 8, 8); // silly request but should be safe
    assert(pk == 0);
    pk = slvmem_rx_peek_bytes(pkbuf, INT_MAX, 8, 8); // silly request but should be safe
    assert(pk == 0);

    rx = slvmem_rx_read_bytes(rxbuf, 8, INT_MAX);
    assert(rx == INT_MIN); // error
    assert(sizeof(rxbuf) > xfer_buffer_rx_capacity());
    rx = slvmem_rx_read_bytes(rxbuf, 8, sizeof(rxbuf));
    assert(rx == INT_MIN); // error
    rx = slvmem_rx_read_bytes(rxbuf, 8, 0);
    assert(rx == 8); // removed

    pk = slvmem_rx_peek_bytes(pkbuf, 0, 1, 0);
    assert(pk == 0); // empty
    rx = slvmem_rx_read_bytes(rxbuf, 1, 0);
    assert(rx == 0); // empty

    /////////////////////////////////////////////////////////////////////////////////

    rx_report();
    breakpoint();
    tx_report();

    auto de1 = slvmem_i2c_deinitialize(init_mode);
    assert(de1);

    fmt::print("#### TEST my_test002() = {}\n", true);

    return true;
}

// init/deinit tests with various modes
bool
my_test003(void)
{
    fake_rp2040_reset();

    auto start = get_absolute_time(); // also sets EPOCH

    struct i2c_slave_config const cfg_values = {
       .sda_pin = 2,
       .scl_pin = 3,
       .address = 0x51, // see example.py
       .baudrate = 200000,
       .use_pullups = 0,
       .nack_mode = NACK_M_NONE
    };
    uint32_t init_mode = INIT_M_DEFAULT;
    auto r0 = slvmem_i2c_initialize(&cfg_values, init_mode);
    assert(r0 == 200000);
    auto r1 = slvmem_i2c_initialize(&cfg_values, init_mode);
    assert(r1 == INT_MIN);

    auto r2 = slvmem_i2c_initialize(NULL, init_mode); // try reinit
    assert(r2 == INT_MIN); // fail

    //////////////////////////////////////////////////////////////////////////////////

    auto de1 = slvmem_i2c_deinitialize(init_mode);
    assert(de1);

    auto r3 = slvmem_i2c_initialize(NULL, init_mode); // try reinit (again)
    assert(r3 == 200000); // success

    //////////////////////////////////////////////////////////////////////////////////

    auto de2 = slvmem_i2c_deinitialize(init_mode);
    assert(de2);

    auto r4 = slvmem_i2c_initialize(&cfg_values, 0); // init_mode==0
    assert(r4 == 200000); // success

    kick_i2c_bogus_handler(rand());
    kick_i2c_bogus_handler(4);

    //////////////////////////////////////////////////////////////////////////////////

    auto de9 = slvmem_i2c_deinitialize(init_mode);
    assert(de9);

    fmt::print("#### TEST my_test003() = {}\n", true);

    return true;
}

// pumping tx data mode
bool
my_test004(void)
{
    fake_rp2040_reset();

    auto start = get_absolute_time(); // also sets EPOCH

    struct i2c_slave_config const cfg_values = {
       .sda_pin = 2,
       .scl_pin = 3,
       .address = 0x51, // see example.py
       .baudrate = 100000,
       .use_pullups = 0,
       .nack_mode = NACK_M_NONE
    };
    uint32_t init_mode = INIT_M_DEFAULT;
    auto r0 = slvmem_i2c_initialize(&cfg_values, init_mode);
    assert(r0 == 100000);
    auto r1 = slvmem_i2c_initialize(&cfg_values, init_mode);
    assert(r1 == INT_MIN);

    auto r2 = slvmem_i2c_initialize(NULL, init_mode); // try reinit
    assert(r2 == INT_MIN); // fail

    //////////////////////////////////////////////////////////////////////////////////

    size_t total_sent = 0;

    size_t total_sent_limit = 7 * 1024;
    if(getenv("COVERAGE_GCOVR_PARSE_BUG")) {
        fmt::print("COVERAGE_GCOVR_PARSE_BUG is enabled\n");
        total_sent_limit = 4 * 1024; // GCOVR reports generation issues
    }

    // simulate trying to output large amounts of data
    for(auto i = 0u; i < 10000; ++i) {
        // simulate a push/drain of all output
        auto txd = slvmem_tx_drain_timeout_us(1000);
        assert(txd >= 0);
        size_t pktsent = 0;
        for(auto n = 0; n < txd; ++n) {
            auto left = slvmem_tx_drain_timeout_us(1000);
            if(left == 0)
                break;
            size_t bytes = rand() % I2CSLAVE_TX_PACKET_LEN; // fuzzing the ISR number of bytes
            bytes = std::max(I2CSLAVE_TX_PACKET_LEN - pktsent, bytes);
            kick_i2c_request_handler(bytes, bytes);
            pktsent += bytes;

            assert(pktsent <= I2CSLAVE_TX_PACKET_LEN);
            if(pktsent == I2CSLAVE_TX_PACKET_LEN) {
                kick_i2c_finish_handler();  // end of I2C message
                pktsent = 0;
            }
        }
        if(pktsent > 0) { // make it a short packet
            kick_i2c_finish_handler();  // end of I2C message
        }

        if(total_sent >= total_sent_limit) // making this larger seems to break gcovr
            break;

        {
            uint8_t txbuf[4096];
            size_t len = rand() % sizeof(txbuf);
            DEBUG_FILL(txbuf, sizeof(txbuf));
            uint8_t const* ptr = ((rand() % 100) < 3) ? nullptr : txbuf; // 3% of time make use nullptr
            size_t minwrite = ((rand() % 100) < 3) ? I2CSLAVE_TX_PACKET_LEN :
                              ((rand() % 100) < 9) ? (rand() % std::min(len, (size_t)I2CSLAVE_TX_MEMBUF_LEN)) : 0;
            assert(minwrite <= I2CSLAVE_TX_MEMBUF_LEN);
            auto tx = slvmem_tx_write_bytes(ptr, len, minwrite);
            assert(tx >= 0);
            fmt::print("### TX: {}\n", ptr ? hexbytes(txbuf, len) : "(null)");
            assert(tx == (int)std::min((int)len, I2CSLAVE_TX_MEMBUF_LEN));

            total_sent += tx;
        }
    }

    fmt::print("#### TOTAL_SENT: {}\n", total_sent);

    //////////////////////////////////////////////////////////////////////////////////

    auto de9 = slvmem_i2c_deinitialize(init_mode);
    assert(de9);

    fmt::print("#### TEST my_test004() = {}\n", true);

    return true;
}

TEST(I2CSlaveTest, AllTheTests) {
  setvbuf(stdout, NULL, _IONBF, 0);
  setvbuf(stderr, NULL, _IONBF, 0);

  time_t now = time(nullptr);
  fmt::print("RANDOM = {}\n", now);
  srand(now);

  EXPECT_TRUE(my_test001());
  EXPECT_TRUE(my_test002());
  EXPECT_TRUE(my_test003());
  EXPECT_TRUE(my_test004());
}
