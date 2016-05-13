'''
    UbloxGPS.py
    
    Description: Library for the Ublox M8N GPS
    
    Revision History
    17 Apr 2016 - V1.0 Created and debugged
    
    Author: Lars Soltmann
    
    INPUTS:    = None (spidev address is 0,0)
    
    
    OUTPUTS:   = gps_lat
               = gps_lon
               = gps_h
               = gps_hmsl
               = gps_stat
               = gps_N
               = gps_E
               = gps_D
               = gps_crs
               = gps_nsat
               = gps_pdop
               = gps_velacc
               = gps_altacc
               = gps_horizacc
    
    NOTES:
    - Written for python3
    - Written for use with Navio2/UBLOX NEO-M8N
    - Checksum is ignored in this code to keep computational time down
    
    '''


import spidev
import math
import struct
import time

class Ublox:
    def __init__(self):
        self.bus = spidev.SpiDev()
        self.bus.open(0,0)  #Specifically for Navio2
    
    def disableNMEA_GLL(self):
        msg = [0xb5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B]
        self.bus.xfer2(msg)
        return None
    
    def disableNMEA_GGA(self):
        msg = [0xb5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x24]
        self.bus.xfer2(msg)
        return None
    
    def disableNMEA_GSA(self):
        msg = [0xb5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32]
        self.bus.xfer2(msg)
        return None
    
    def disableNMEA_GSV(self):
        msg = [0xb5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39]
        self.bus.xfer2(msg)
        return None
    
    def disableNMEA_RMC(self):
        msg = [0xb5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40]
        self.bus.xfer2(msg)
        return None
    
    def disableNMEA_VTG(self):
        msg = [0xb5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47]
        self.bus.xfer2(msg)
        return None
    
    def GNSS_Reset(self):
        msg = [0xb5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00, 0x02, 0x00, 0x10, 0x68]
        self.bus.xfer2(msg)
        return None
    
    def setNavEngine(self):
        #Platform - 0 - portable
        #UBlox8 ONLY - StaticHoldMaxDist-200
        #msg = [0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0xC8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 0x5C]
        
        #Platform - 6 - airborned <1g
        #UBlox8 ONLY - StaticHoldMaxDist-200
        #msg = [0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0xC8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1A, 0x28]
        
        #Platform - 7 - airborned <2g
        #UBlox8 ONLY - StaticHoldMaxDist-200
        msg = [0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0xC8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1B, 0x4A]
        
        #Platform - 8 - airborned <4g
        #UBlox8 ONLY - StaticHoldMaxDist-200
        #msg = [0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x08, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0xC8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x6C]
        
        
        #Platform - 0 - portable
        #UBlox7 or 8
        #msg = [0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4C, 0x1C]
        
        #Platform - 6 - airborned <1g
        #UBlox7 or 8
        #msg = [0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x52, 0xE8]
        
        #Platform - 7 - airborned <2g
        #UBlox7 or 8
        #msg = [0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x53, 0x0A]
        
        #Platform - 8 - airborned <4g
        #UBlox7 or 8
        #msg = [0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x08, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x54, 0x2C]

        self.bus.xfer2(msg)
        return None
    
    def setRATE(self):
        #msg = [0xb5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xFA, 0x00, 0x01, 0x00, 0x01, 0x00, 0x10, 0x96] #4Hz
        msg = [0xb5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A] #5Hz
        self.bus.xfer2(msg)
        return None
    
    def enableNAV_PVT(self):
        msg = [0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x07, 0x01, 0x13, 0x51]
        self.bus.xfer2(msg)
        return None
               
    def initialize(self):
        #Disable default NMEA messages
        self.disableNMEA_GLL()
        time.sleep(0.01)
        self.disableNMEA_GGA()
        time.sleep(0.01)
        self.disableNMEA_GSA()
        time.sleep(0.01) 
        self.disableNMEA_GSV()
        time.sleep(0.01)        
        self.disableNMEA_RMC()
        time.sleep(0.01)        
        self.disableNMEA_VTG()
        time.sleep(0.01)

        #Setup GPS       
        self.enableNAV_PVT()
        time.sleep(0.01)
        self.setNavEngine()
        time.sleep(0.01)
        self.setRATE()
        time.sleep(0.01)
        
        print('Initialization complete.')

        return None
               
    def getMessages(self):
        message_ID=0;
        to_gps_data = [0x00]
        from_gps_data = [0x00]
        data_array=[0]*100 # 100 bytes is the size of the PVT message (92+8)
        message_flag=0
        self.typeFlag=0
        spi_transfer_data_length=1

        while True:
            for i in range(spi_transfer_data_length):
                from_gps_data=self.bus.xfer2(to_gps_data)
                from_gps_data=from_gps_data[0]
                data_array[i+4]=from_gps_data

            if message_flag==0:
                message_ID=self.messageType(from_gps_data, data_array)
                if message_ID==0x03:
                    spi_transfer_data_length=20
                    message_flag=1
                elif message_ID==0x02:
                    spi_transfer_data_length=32
                    message_flag=1
                elif message_ID==0x12:
                    spi_transfer_data_length=40
                    message_flag=1
                elif message_ID==0x07:
                    spi_transfer_data_length=96
                    message_flag=1

            elif message_flag==1:
                self.decodeMessage(data_array)
                message_flag=0
                message_ID=0
                spi_transfer_data_length=1
                data_array[0:4]=[0,0,0,0]
                self.typeFlag=0

            if message_flag==0:
                time.sleep(0.0001)

        return None
               
               

    def messageType(self,from_gps_data,data_array):
        if (from_gps_data==0xb5 and self.typeFlag==0):
            self.typeFlag=1
            data_array[0]=from_gps_data
            message_ID=0
        elif (from_gps_data==0x62 and self.typeFlag==1):
            self.typeFlag=2
            data_array[1]=from_gps_data
            message_ID=0
        elif (self.typeFlag==2):
            self.typeFlag=3
            data_array[2]=from_gps_data
            message_ID=0
        elif (self.typeFlag==3):
            message_ID=from_gps_data;
            self.typeFlag=0;
            data_array[3]=message_ID
        else:
            self.typeFlag=0
            message_ID=0

        return message_ID

               
            
    def decodeMessage(self,data_array):
        if (data_array[3]==0x07):
            #                           012345    1    5    2    5    3    5    4
            curr_values=struct.unpack("<BBBBHIHBBBBBBIiBBBBiiiiIIiiiiiIIHBBBBBBiBBBBH", bytearray(data_array))
            self.gps_lat=curr_values[20]*0.0000001 #deg
            self.gps_lon=curr_values[19]*0.0000001 #deg
            self.gps_h=curr_values[21]*0.00328084  #mm to ft
            self.gps_hmsl=curr_values[22]*0.00328084 #mm to ft
            self.gps_stat=curr_values[15]
            self.gps_N=curr_values[25]*0.00328084 #mm/s to ft/s
            self.gps_E=curr_values[26]*0.00328084 #mm/s to ft/s
            self.gps_D=curr_values[27]*0.00328084 #mm/s to ft/s
            # GPS 2D and 3D velocity will be calculated during post processing
            self.gps_crs=curr_values[29]*0.00001 #deg
            self.gps_nsat=curr_values[18] #no units
            self.gps_pdop=curr_values[32]*0.01 #no units
            self.gps_velacc=curr_values[30]*0.00328084 #mm/s to ft/s
            self.gps_altacc=curr_values[24]*0.00328084 #mm to ft
            self.gps_horizacc=curr_values[23]*0.00328084 #mm to ft
        else:
            self.gps_lat=0
            self.gps_lon=0
            self.gps_h=0
            self.gps_hmsl=0
            self.gps_stat=0
            self.gps_N=0
            self.gps_E=0
            self.gps_D=0
            self.gps_crs=0
            self.gps_nsat=0
            self.gps_pdop=0
            self.gps_velacc=0
            self.gps_altacc=0
            self.gps_horizacc=0
               

        #Debug
        #print('Lat | Lon:   %.6f %.6f' % (self.gps_lat,self.gps_lon))
        #print('NED vel:     %.2f %.2f %.2f' % (self.gps_N,self.gps_E,self.gps_D))
       	#print('h | hmsl:    %.2f %.2f' % (self.gps_h,self.gps_hmsl))
        #print('Course:      %.2f' % (self.gps_crs))
        #print('Status:      %d' % (self.gps_stat))
        #print('Satellites:  %d' % (self.gps_nsat))
        #print('PDOP:        %.2f\n' % (self.gps_pdop))

        return None
