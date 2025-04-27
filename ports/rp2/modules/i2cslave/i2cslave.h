// modules/i2cslave/i2cslave.h
#ifndef MICROPY_INCLUDED_I2CSLAVE_H
#define MICROPY_INCLUDED_I2CSLAVE_H
#include "py/qstr.h"

// Declare qstrs for the module and its functions
#define MP_QSTR_i2cslave (MP_QSTR_ ## i2cslave)
#define MP_QSTR_write_bytes (MP_QSTR_ ## write_bytes)
#define MP_QSTR_read_bytes (MP_QSTR_ ## read_bytes)

#endif // MICROPY_INCLUDED_I2CSLAVE_H
