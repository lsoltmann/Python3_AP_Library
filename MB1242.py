'''
    MB1242.py
    
    Description: Library for the MaxBotix MB1242 I2CXL-MaxSonar-EZ4
    
    Revision History
    17 Apr 2016 - Created and debugged
    
    Author: Lars Soltmann
    
    INPUTS:     i2c_addr = I2C address of sensor
    
    OUTPUTS:    self.dist = Measured sensor distance (cm)
    
    NOTES:
    - Written for python3
    - MaxBotix recommends waiting 100ms between readings
    
    '''

import time
import smbus

class MB1242:
    def __init__(self, i2c_addr):
        self.addr=i2c_addr
        self.bus=smbus.SMBus(1)

    def refreshDistance(self):
        self.bus.write_byte(self.addr, 0x51)
        return None
        
    def readDistance(self):   
        dist_uf=self.bus.read_i2c_block_data(self.addr,0x00)
        self.dist=float((dist_uf[0] * 256) + dist_uf[1])
        return None
