/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2025 Pat Deegan, https://psychogenic.com
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

#define I2CSLAVE_MEMBUF_LEN (16*8)
#define I2CSLAVE_DEVICE i2c1


typedef struct
{
    uint8_t mem[I2CSLAVE_MEMBUF_LEN];
    uint8_t mem_index;
    uint8_t mem_len;
} xfer_buffer;


typedef void(*data_in_callback)(uint8_t len, uint8_t *bts);
typedef void(*data_out_done_callback)(void);

void slvmem_i2c_init(uint8_t sda_pin, uint8_t scl_pin, 
    uint8_t address,
    uint baudrate,
    data_in_callback cb_datain,
    data_out_done_callback cb_dataout_done,
    uint8_t use_pullups
);

void slvmem_set_callbacks(
    data_in_callback cb_datain,
    data_out_done_callback cb_dataout_done
);

void slvmem_flush_output(void);


void slvmem_set_data_out(uint8_t len, uint8_t * bts);


void slvmem_i2c_deinit(void);
