'''
    MS5805.py
    
    Description: Driver for MS5805 pressure transducer
    
    Revision History
    05 May 2017 - Created and debugged
    
    Author: Lars Soltmann
    
    References: MS5805 data sheet
    
    Notes:
    - Written for Python3
    
    Hardware Requirements:
    - MS5805 pressure transducer
    
    Calls: None
    
    Open Items:
    1. None
    
'''

import smbus
import time
import math

class MS5805:
    def __init__(self,devAddr):
        self.devAddr=devAddr
        self.bus=smbus.SMBus(1)
        self.D1=0x4A
        self.D2=0x5A
        self.RESET=0x1E
        self.ADCREAD=0x00
        self.CRC=0xA0
        self.CAL_C1=0xA2
        self.CAL_C2=0xA4
        self.CAL_C3=0xA6
        self.CAL_C4=0xA8
        self.CAL_C5=0xAA
        self.CAL_C6=0xAE
        #MS5805_DEFAULT_ADDRESS 0x76
        
    def initialize(self):
        # Write the reset command to get the calibration coefficients
        self.bus.write_i2c_block_data(self.devAddr, self.RESET,[0])
        # Read the calibration coefficients
        temp_val=self.bus.read_i2c_block_data(self.devAddr, self.CAL_C1,2)
        self.C1 = temp_val[0]<<8 | temp_val[1]
        temp_val=self.bus.read_i2c_block_data(self.devAddr, self.CAL_C2,2)
        self.C2 = temp_val[0]<<8 | temp_val[1]
        temp_val=self.bus.read_i2c_block_data(self.devAddr, self.CAL_C3,2)
        self.C3 = temp_val[0]<<8 | temp_val[1]
        temp_val=self.bus.read_i2c_block_data(self.devAddr, self.CAL_C4,2)
        self.C4 = temp_val[0]<<8 | temp_val[1]
        temp_val=self.bus.read_i2c_block_data(self.devAddr, self.CAL_C5,2)
        self.C5 = temp_val[0]<<8 | temp_val[1]
        #temp_val=self.bus.read_i2c_block_data(self.devAddr, self.CAL_C6,2)
        #self.C6 = temp_val[0]<<8 | temp_val[1]
        
        # For the 6th coefficient, force it to the given value since it doesn't seem to read correctly
        # *!*!*!*!*!*!*!*!*!*!
        self.C6 = 27058;
        # *!*!*!*!*!*!*!*!*!*!


    def read_pressure_temperature(self):
        self.bus.write_i2c_block_data(self.devAddr, self.D1,[0])
        time.sleep(0.02)
        temp_val=self.bus.read_i2c_block_data(self.devAddr, self.ADCREAD,3)
        pressi = (temp_val[0] << 16) | (temp_val[1] << 8) | temp_val[2]
        
        self.bus.write_i2c_block_data(self.devAddr, self.D2,[0])
        time.sleep(0.02)
        temp_val=self.bus.read_i2c_block_data(self.devAddr, self.ADCREAD,3)
        tempi = (temp_val[0] << 16) | (temp_val[1] << 8) | temp_val[2]
        
        dT = tempi - self.C5 * 2**8
        temp = (2000 + ((dT * self.C6) / 2**23))
        
        OFF = self.C2 * 2**17 + (self.C4 * dT) / 2**6
        SENS = self.C1 * 2**16 + (self.C3 * dT) / 2**7
        
        
        if (temp >= 2000):
            T2 = 0;
            OFF2 = 0;
            SENS2 = 0;
        elif (temp < 2000):
            T2 = 11 * dT * dT / 2**35
            OFF2 = 31 * ((temp - 2000)**2) / 2**3
            SENS2 = 63 * ((temp - 2000)**2) / 2**5
        
        temp = temp - T2
        OFF = OFF - OFF2
        SENS = SENS - SENS2
        
        # Final calculations
        self.PRESS = ((pressi * SENS) / 2**21 - OFF) / 2**15 / 100
        self.TEMP = temp / 100


    def getTemperature_degF(self):
        return self.TEMP*1.8+32

    def getTemperature_egC(self):
        return self.TEMP 


    def getPressure_mbar(self):
        return self.PRESS

    def getPressure_psf(self):
        return self.PRESS*2.0885434273
