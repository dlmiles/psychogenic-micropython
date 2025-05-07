/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2025 Darryl L. Miles
 *
 */

#ifndef __FAKE_RP2040_API_TYPES_H
#define __FAKE_RP2040_API_TYPES_H

#include <stdbool.h>
#include <stdint.h>
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif


typedef uint64_t absolute_time_t;
typedef uint32_t uint;

// Dummy I2C hardware handle/state/struct
typedef struct i2c_inst {
    void* dummy;
    bool has_inited;
    bool is_slave;

    size_t rx_cap;
    size_t rx_avail;
    size_t rx_pos;
    uint8_t rxbuf[4096];

    size_t tx_cap;
    size_t tx_space;
    size_t tx_pos;
    uint8_t txbuf[4096];
} i2c_inst_t;

typedef struct spin_lock {
    void* dummy;
    pthread_spinlock_t lock;
    int value;
    bool claimed;
    uint32_t saved_irq;
} spin_lock_t;


enum gpio_function { GPIO_FUNC_I2C = 3 };

typedef enum gpio_function gpio_function_t;


enum i2c_slave_event { I2C_SLAVE_RECEIVE, I2C_SLAVE_REQUEST, I2C_SLAVE_FINISH };

typedef enum i2c_slave_event i2c_slave_event_t;

typedef void(*i2c_slave_handler_t)(i2c_inst_t* i2c, i2c_slave_event_t event);


#ifdef __cplusplus
}
#endif

#endif // __FAKE_RP2040_API_TYPES_H
