/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2025 Darryl L. Miles
 *
 */

#ifndef __TEST_DEBUGLOG_H
#define __TEST_DEBUGLOG_H

#include "./lib/i2cslave_sock_seqpacket.h"

#ifdef __cplusplus
#include <string>

extern "C" void hexbyte(char* buf, uint8_t b);

extern std::string dump_data_to_string(void const* data, size_t len);

extern std::string hexbytes(void const* data, size_t len);

#endif


#ifdef __cplusplus
extern "C" {
#endif

#ifdef TEST_WITH_FAKE_RP2040

//typedef struct xfer_buffer xfer_buffer_t; /* forward decl */
//typedef struct xfer_buffer_state xfer_buffer_state_t; /* forward decl */

extern const char* recv_mem_operation_to_string(const struct recv_mem_operation* p);

extern const char* send_mem_operation_to_string(const struct send_mem_operation* p);

extern const char* xfer_buffer_state_to_string(const xfer_buffer_state_t* st);

extern const char* xfer_buffer_to_string(const xfer_buffer_t* xb);

extern const char* i2c_slave_config_to_string(const i2c_slave_config_t* cfg);

extern void DEBUG_ENTRY(const char* fmt, ...) __attribute__((format (printf, 1, 2)));

extern void DEBUG_RETURN(const char* fmt, ...) __attribute__((format (printf, 1, 2)));

extern void DEBUG_MESSAGE(const char* fmt, ...) __attribute__((format (printf, 1, 2)));

extern void DEBUG_FILL(void* p, size_t len);

extern bool assert_buffer(const xfer_buffer_t* xb);

extern bool assert_state_aligned(const xfer_buffer_t* st, bool aligned_mem_in, bool aligned_mem_out);

extern bool assert_state_aligned_mem_in(const xfer_buffer_t* st);

extern bool assert_state_aligned_mem_out(const xfer_buffer_t* st);

extern bool assert_state(const xfer_buffer_t* st);

#define DEBUG_ASSERT(cond)  assert(cond)

#else

// FIXME This section got copied into driver, clean that up
#define DEBUG_ENTRY(fmt, ...) do {} while(0)
#define DEBUG_RETURN(fmt, ...) do {} while(0)
#define DEBUG_MESSAGE(fmt, ...) do {} while(0)

#define DEBUG_ASSERT(cond)  /* */
#define DEBUG_FILL(ptr, len) /* */

#endif

#ifdef __cplusplus
}
#endif

#endif // __TEST_DEBUGLOG_H
