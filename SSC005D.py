'''
    SSC005D.py
    
    Description: Driver for Honeywell SSCDRRN005ND2A5 pressure transducer,
                 although the library can be used for any Honeywell digital I2C pressure transducer
    
    Revision History
    05 May 2017 - Created and debugged
    
    Author: Lars Soltmann
    
    References: SSCDRRN005ND2A5 data sheet
    
    Notes:
    - Written for Python3
    
    
    Hardware Requirements:
    - SSCDRRN005ND2A5 or any Honeywell digital I2C pressure transducer
    
    Calls: None
    
    
    Open Items:
    1. None
    
'''

import smbus
import time
import math

class HWSSC:
    def __init__(self,devAddr):
        self.devAddr=devAddr
        self.bus=smbus.SMBus(1)


    def readPressure_raw(self):
        # Read two bytes of data
        pdata=self.bus.read_i2c_block_data(self.devAddr, 0,2)
        # The status byte is the first two bits of the MSB
        #status=pdata_arr[0] >> 6
        # Combine the two bytes
        pdata=((pdata[0]<<8)+pdata[1])
        # Set the status bits equal to zero
        self.pdata=pdata & 0x3fff
        return None

    def convertPressure(self,calRange,sensRange):
        if (calRange == 1): # 10 to 90% calibration
            press=(1.525878906e-4*self.pdata-1.25)*sensRange
        
        elif (calRange == 2): # 5 to 95% calibration
            press=(1.356336806e-4*self.pdata-1.111111111)*sensRange

        elif (calRange == 3): # 5 to 85% calibration
            press=(1.525878906e-4*self.pdata-1.125)*sensRange

        elif (calRange == 4): # 4 to 94% calibration
            press=(1.356336806e-4*self.pdata-1.088888889)*sensRange
        
        return press
