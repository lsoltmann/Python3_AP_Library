'''
    Read_Config.py
    
    Description: Read configuration/calibration files
    
    Revision History
    17 Mar 2016 - Created and debugged
    14 Apr 2016 - Updated
    28 Apr 2016 - Added reading of magnetometer calibration file
    
    Author: Lars Soltmann
    
    
    File format for configuration file:
    ______________________________
    #Configuration file
    #
    #General format for file is:
    #'Variable'
    #<values>
    #
    #For PID variables the value line format is:
    #kp kd ki abs(integral_limit)
    
    Pitch PID
    1.1 2.2 3.3 1.2

    Roll PID
    4.4 5.5 6.6 4.5

    Yaw PID
    7.7 8.8 9.9 7.8
    ______________________________
    
    
    Output:
        p_pitch         = porportional gain, pitch
        d_pitch         = derivative gain, pitch
        i_pitch         = integral gain, pitch
        il_pitch        = integral limit, pitch
        
        p_roll          = porportional gain, roll
        d_roll          = derivative gain, roll
        i_roll          = integral gain, roll
        il_roll         = integral limit, roll
        
        p_yaw           = porportional gain, yaw
        d_yaw           = derivative gain, yaw
        i_yaw           = integral gain, yaw
        il_yaw          = integral limit, yaw
        
        PWM_MIN         = minimum PWM output value (u_sec)
        PWM_MAX         = maximum PWM output value (u_sec)
        PWM_FREQ        = PWM pulse frequency (Hz)

        max_p           = maximum pitch angle (deg)
        max_r           = maximum roll angle (deg)
        max_dy          = maximum yaw rate(deg)

        dead_band       = one-sided PWM range around neutral where control input is set to zero

        thr_cut         = one-sided PWM range above minimum throttle for which all motors reamin off

        FROM CALIBRATION FILE

        Pm              = slope for converting from PWM to angle, pitch
        Pbl             = lower range intercept for converting from PWM to angle, pitch
        Pbh             = upper range intercept for converting from PWM to angle, pitch
        Rm              = slope for converting from PWM to angle, roll
        Rbl             = lower range intercept for converting from PWM to angle, roll
        Rbh             = upper range intercept for converting from PWM to angle, roll
        Ym              = slope for converting from PWM to rate, yaw
        Ybl             = lower range intercept for converting from PWM to rate, yaw
        Ybh             = lower range intercept for converting from PWM to rate, yaw
        
        Pcn             = center PWM value of elevator stick (u_sec)
        Rcn             = center PWM value of aileron stick (u_sec)
        Ycn             = center PWM value of rudder stick (u_sec)
    
        Tmin            = minimum recorded throttle value
    
'''

import sys

class read_config_file:
    def __init__(self,file_name1,file_name2,file_name3):
        self.file_name1=file_name1
        self.file_name2=file_name2
        self.file_name3=file_name3
        self.flag=0

    def read_configuration_file(self):
        #Open the config file
        file1 = open(self.file_name1, 'r')
        for line in file1:
            #Skip lines that are blank or start with
            if (line[0]=='#' or line=='\n'):
                pass
            
            #### PITCH_PID INFORMATION (1)
            #If this phrase is found, set a flag so that the next time around the line is read into the appropriate variables
            elif line=='PITCH_PID\n' or self.flag==1:
                if self.flag==1:
                    current_line=[float(s) for s in line.split()]
                    #The values line should be 4 numbers following the format above, if not, then exit with error
                    if len(current_line)==4:
                        self.p_pitch,self.d_pitch,self.i_pitch,self.il_pitch=[float(s) for s in line.split()]
                        #DEBUG
                        #print(self.p_pitch)
                        #print(self.d_pitch)
                        #print(self.i_pitch)
                        #print(self.il_pitch)
                    else:
                        sys.exit('Error reading PITCH_PID values!')
                    self.flag=0
                else:
                    self.flag=1
        
            #### ROLL_PID INFORMATION (2)
            elif line=='ROLL_PID\n' or self.flag==2:
                 if self.flag==2:
                     current_line=[float(s) for s in line.split()]
                     if len(current_line)==4:
                         self.p_roll,self.d_roll,self.i_roll,self.il_roll=[float(s) for s in line.split()]
                         #DEBUG
                         #print(self.p_roll)
                         #print(self.d_roll)
                         #print(self.i_roll)
                         #print(self.il_roll)
                     else:
                         sys.exit('Error reading ROLL_PID values!')
                     self.flag=0
                 else:
                     self.flag=2
                         
            #### YAW_PID INFORMATION (3)
            elif line=='YAW_PID\n' or self.flag==3:
                 if self.flag==3:
                     current_line=[float(s) for s in line.split()]
                     if len(current_line)==4:
                         self.p_yaw,self.d_yaw,self.i_yaw,self.il_yaw=[float(s) for s in line.split()]
                         #DEBUG
                         #print(self.p_yaw)
                         #print(self.d_yaw)
                         #print(self.i_yaw)
                         #print(self.il_yaw)
                     else:
                         sys.exit('Error reading YAW_PID values!')
                     self.flag=0
                 else:
                     self.flag=3
                     
            #### MAX_PITCH INFORMATION (4)
            elif line=='MAX_PITCH\n' or self.flag==4:
                if self.flag==4:
                    current_line=[float(s) for s in line.split()]
                    if len(current_line)==1:
                        self.max_p=current_line[0]
                        #DEBUG
                        #print(self.max_p)
                    else:
                        sys.exit('Error reading MAX_PITCH value!')
                    self.flag=0
                else:
                    self.flag=4

            #### MAX_ROLL INFORMATION (5)
            elif line=='MAX_ROLL\n' or self.flag==5:
                if self.flag==5:
                    current_line=[float(s) for s in line.split()]
                    if len(current_line)==1:
                        self.max_r=current_line[0]
                        #DEBUG
                        #print(self.max_r)
                    else:
                        sys.exit('Error reading MAX_ROLL value!')
                    self.flag=0
                else:
                    self.flag=5

            #### MAX_YAWRATE INFORMATION (6)
            elif line=='MAX_YAWRATE\n' or self.flag==6:
                if self.flag==6:
                    current_line=[float(s) for s in line.split()]
                    if len(current_line)==1:
                        self.max_dy=current_line[0]
                        #DEBUG
                        #print(self.max_dy)
                    else:
                        sys.exit('Error reading MAX_YAWRATE value!')
                    self.flag=0
                else:
                    self.flag=6
            

            #### PWM_RANGE INFORMATION (7)
            elif line=='PWM_RANGE\n' or self.flag==7:
                 if self.flag==7:
                     current_line=[float(s) for s in line.split()]
                     if len(current_line)==2:
                         self.PWM_MIN,self.PWM_MAX=[float(s) for s in line.split()]
                         #DEBUG
                         #print(self.PWM_MIN)
                         #print(self.PWM_MAX)
                     else:
                         sys.exit('Error reading PWM_RANGE value!')
                     self.flag=0
                 else:
                     self.flag=7
        
            #### PWM_FREQ INFORMATION (8)
            elif line=='PWM_FREQ\n' or self.flag==8:
                 if self.flag==8:
                     current_line=[float(s) for s in line.split()]
                     if len(current_line)==1:
                         self.PWM_FREQ=current_line[0]
                         #DEBUG
                         #print(self.PWM_FREQ)
                     else:
                         sys.exit('Error reading PWM_FREQ value!')
                     self.flag=0
                 else:
                     self.flag=8
                         
            #### DEAD_BAND INFORMATION (9)
            elif line=='DEAD_BAND\n' or self.flag==9:
                 if self.flag==9:
                     current_line=[float(s) for s in line.split()]
                     if len(current_line)==1:
                         self.dead_band=current_line[0]
                         #DEBUG
                         #print(self.dead_band)
                     else:
                         sys.exit('Error reading DEAD_BAND value!')
                     self.flag=0
                 else:
                     self.flag=9
                     
            #### DEAD_BAND INFORMATION (10)
            elif line=='THR_CUT\n' or self.flag==10:
                 if self.flag==10:
                     current_line=[float(s) for s in line.split()]
                     if len(current_line)==1:
                         self.thr_cut=current_line[0]
                         #DEBUG
                         #print(self.thr_cut)
                     else:
                         sys.exit('Error reading THR_CUT value!')
                     self.flag=0
                 else:
                     self.flag=10

            else:
                sys.exit('Unknown error reading configuration file!')
        
        file1.close()
        return None

    def read_calibration_file(self):
        #Open the config file
        try:
            file2 = open(self.file_name2, 'r')
            data_uf=file2.read()
            data=[float(s) for s in data_uf.split()]
            self.cmax_r=data[0]
            self.cmax_p=data[1]
            self.cmax_dy=data[2]
            self.cdead_band=data[3]
            self.Rcn=data[4]
            self.Pcn=data[5]
            self.Ycn=data[7]
            self.Tmin=data[10]
            self.Rm=data[16]
            self.Rbl=data[17]
            self.Rbh=data[18]
            self.Pm=data[19]
            self.Pbl=data[20]
            self.Pbh=data[21]
            self.Ym=data[22]
            self.Ybl=data[23]
            self.Ybh=data[24]
            file2.close()
            return None

        except:
            sys.exit('No calibration file found! Control calibration (mode=3) necessary.')

    def calibration_check(self):
        #Check to make sure that calibration file data and configuration file data match.
        #If they don't, exit program and tell user to perform control calibration.
        if ((self.cmax_p != self.max_p) or (self.cmax_r != self.max_r) or (self.cmax_dy != self.max_dy) or (self.cdead_band != self.dead_band)):
            sys.exit('Calibration data does not match configuration data! Control calibration (mode=3) necessary.')
        else:
            pass
        return None

    def read_magnetometer_calibration_file(self):
        #Open the mag_calib file
        try:
            file3 = open(self.file_name3, 'r')
            data_uf=file3.read()
            data=[float(s) for s in data_uf.split()]
            self.hix=data[0]
            self.hiy=data[1]
            self.hiz=data[2]
            file3.close()
            return None

        except:
            sys.exit('No magnetometer calibration file found! Run mangetometer calibration script.')
