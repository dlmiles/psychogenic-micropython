'''
    Copyright (C) 2025 Pat Deegan, https://psychogenic.com
    
    Simple example of an implementation class that lets you
    
    * setup the i2c slave
    
    * call queue_data() as much as you like for outgoing
    
    * get a callback to printout incoming

'''
SlaveAddy = 0x51

import machine
import i2cslave

class I2CDevice:
    SlaveBufferSize = 16*7
    
    def __init__(self, address:int, scl:int=3, sda:int=2, baudrate:int=100000):
        self._addr = address 
        self._scl = scl 
        self._sda = sda 
        self._baud = baudrate
        self._dataqueue = bytearray()
        self._slavebuf_filled = False
        
    def data_received(self, numbytes:int, bts:bytearray):
        print(f'Data in: {bts}')
        
        
    def _write_outbytes(self):
        if self._slavebuf_filled or not len(self._dataqueue):
            return None
        # 
        if len(self._dataqueue) > self.SlaveBufferSize:
            to_send = self._dataqueue[:self.SlaveBufferSize]
            self._dataqueue = self._dataqueue[self.SlaveBufferSize:]
        else:
            to_send = self._dataqueue
            self._dataqueue = bytearray()
            
        i2cslave.write_bytes(len(to_send), to_send)
        self._slavebuf_filled = True
        return len(to_send)
        
    def queue_outdata(self, data_out:bytearray):
        self._dataqueue += data_out
        if not len(self._dataqueue):
            return 
            
        self._write_outbytes()
        
    def flush_output(self):
        self._dataqueue = bytearray()
        i2cslave.flush_output()
        
    
    def _data_tx_done_cb(self):
        self._slavebuf_filled = False
        self._write_outbytes()
        
    def begin(self):
        i2cslave.setup(self._addr, self._scl, self._sda, self._baud) 
        i2cslave.set_datain_callback(self.data_received)
        i2cslave.set_datatxdone_callback(self._data_tx_done_cb)
        try:
            i2cslave.initialize()
        except:
            print("i2c slave init failed!")
            return False 
            
        return True
        

if __name__ == '__main__':
    import time
    
    i2cs = I2CDevice(SlaveAddy, 3, 2, 100000)
    if i2cs.begin():
        print("I2C device initialized!")
    i2cs.queue_outdata(bytearray(list(range(16*16))))
    
    print('Write data to this device to see it printed.  Read blocks of, say, 16 to see all the queued data')
    
    while True:
        time.sleep(1)



# start_i2c()