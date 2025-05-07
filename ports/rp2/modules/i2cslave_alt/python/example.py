'''
    Copyright (C) 2025 Pat Deegan, https://psychogenic.com

    Simple example of an implementation class that lets you

    * setup the i2c slave

    * call queue_data() as much as you like for outgoing

    * get a callback to printout incoming

'''
SlaveAddy = 0x51

import sys
import machine
import i2cslave_alt as i2cslave

class I2CDevice:
    def __init__(self, address:int, scl:int=3, sda:int=2, baudrate:int=100000, pullup:bool=None):
        self._addr = address
        self._scl = scl
        self._sda = sda
        self._baud = baudrate
        self._pullup = pullup

        self._data = bytearray()

        return None

    def tx_write_bytes(self, data:bytearray = None) -> int:
        if data is not None:
            self._data += data_out
        length = len(self._data)
        if length == 0:
            return

        offset = 0
        length -= offset # relevant when offset>0
        res = i2cslave.tx_write_bytes(self._data, 0, len(self._data))
        if res > 0:
            offset += res
            self._data = self._data[offset+res:] # cut sent bytes out
        return res

    def tx_discard_all(self):
        self._data = bytearray()
        i2cslave.tx_discard_all()

    def tx_drain_timeout_us(self, timeout_us: int) -> int:
        return i2cslave.tx_drain_timeout_us(timeout_us)

    def begin(self):
        i2cslave.setup(self._addr, self._scl, self._sda, self._baud, self._pullup)
        try:
            res = i2cslave.initialize()
            if res > 0:
                print(f"SUCCESS: I2C slave init! SDA={self._sda} SCL={self._scl} ADDR=0x{self._addr:0x} BAUD={self._baud} PU={self._pullup} res={res}")
                self.report_constants()
                self.report()
                self.report_activity()
                return True
        except Exception as ex:
            sys.print_exception(ex)

        print("ERROR: i2c slave init failed!  res={res}")
        return False


if __name__ == '__main__':
    import time

    # i2cslave.setup(SlaveAddy, 3, 2, 100000)
    # i2cslave.initialize()
    # This seems to be an object created for the safe of having a class,
    #  the main value add of the class seems to be bytearray() management
    #  but this seems asking for allocation trouble.
    i2cs = I2CDevice(SlaveAddy, 3, 2, 100000)
    res = i2cs.begin()
    assert(res)
    n = i2cs.tx_write_bytes(bytearray(list(range(16*16)))) # list?
    print(f'tx_write_bytes() = {n}')

    print('Write data to this device to see it printed.  Read blocks of, say, 16 to see all the queued data')

    # This solution is not callback based, it is polling based
    ba = bytearray(8) # reuse this
    while True:
        can_sleep = True

        plen = i2cslave.rx_peek_bytes(ba, 0, len(ba))
        if plen > 0:
            print('peek_bytes() = {} {}'.format(plen, str(ba)))

        rlen = i2cslave.rx_read_bytes(ba, 0, len(ba))
        if rlen > 0:
            print('read_bytes() = {} {}'.format(rlen, str(ba)))
            can_sleep = False # try to process more data

        # Well it is possible the C driver can do a little more (detecting I2C h/w or bus issues to consider a h/w reset)
        res = i2cslave.do_driver_stuff() # NOP right now
        print(f'do_driver_stuff() = {res}')

        if can_sleep:
            notify_abs = i2cslave.get_notify_abs()
            print(f'notify_abs={notify_abs} [{type(notify_abs)}]')
            res = i2cslave.waitfor_notify_timeout_us(notify_abs, 5000000) # 5sec slow loop down a lot
            print(f'notify = {res}')

        bs = i2cslave.get_busy_state()
        print(f'busy_state={bs}')

        # Exercise the API calls
        tx_avail = i2cslave.tx_available()
        tx_space = i2cslave.tx_space()
        tx_capac = i2cslave.tx_capacity()
        tx_grain = i2cslave.tx_granularity()
        tx_avail_raw = i2cslave.tx_available_raw()
        tx_space_raw = i2cslave.tx_space_raw()
        print(f'tx={{avail={tx_avail}, spc={tx_space}, cap={tx_capac}, gra={tx_grain}}} {{tx_ava_r={tx_avail_raw}, tx_spc_r{tx_space_raw}}}')

        rx_avail = i2cslave.rx_available()
        rx_space = i2cslave.rx_space()
        rx_capac = i2cslave.rx_capacity()
        rx_grain = i2cslave.rx_granularity()
        rx_avail_raw = i2cslave.rx_available_raw()
        rx_space_raw = i2cslave.rx_space_raw()
        print(f'rx={{avail={rx_avail}, spc={rx_space}, cap={rx_capac}, gra={rx_grain}}} {{rx_ava_r={rx_avail_raw}, rx_spc_r={rx_space_raw}}}')

        notify_abs = i2cslave.get_notify_abs()
        notify_elapsed = time.time_us() - notify_abs if notify_abs != 0 else 'NEVER'
        print(f'notify_abs={notify_abs} notify_elapsed={notify_elapsed} [{type(notify_abs)}]') # check this is a 64bit number
        timeout_abs = time.time_us() + 500000 # time.get_absolute_time()
        res = i2cslave.get_waitfor_notify_timeout_abs(notify_abs, timeout_abs) # slow loop down a bit
        print(f'get_waitfor_notify() = {res}')
        res = i2cslave.get_waitfor_notify_timeout_us(notify_abs, 0)

        res = i2cslave.tx_trylock_timeout_us(1)
        assert(res > 0)
        i2cslave.tx_unlock()

        res = i2cslave.rx_trylock_timeout_us(1)
        assert(res > 0)
        i2cslave.rx_unlock()

# start_i2c()
