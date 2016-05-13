'''
    MB1242.py
    
    Description: Library for the MaxBotix MB1242 I2CXL-MaxSonar-EZ4
    
    Revision History
    17 Apr 2016 - Created and debugged
    
    Author: Lars Soltmann
    
    INPUTS:     i2c_addr = I2C address of sensor
    
    OUTPUTS:    us_dist = Measured sensor distance
    
    NOTES:
    - Written for python3
    
    '''

import time
import smbus

class MB1242:
    def __init__(self, i2c_addr):
        self.addr=i2c_addr
        self.bus=smbus.SMBus(1)

    def distance(self,wait_time=0.1):
        self.bus.write_byte(self.addr, 0x51)
        time.sleep(wait_time)
        dist_uf=self.bus.read_i2c_block_data(self.addr,0x00)
        #dist_uf=self.bus.read_word_data(self.addr,0x00)
        dist=(dist_uf[0] * 256) + dist_uf[1]
        return dist
