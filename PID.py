'''
    PID.py
    
    Description: PID controller class
    
    Revision History
    17 Mar 2016 - Created and debugged
    18 Apr 2016 - Updated, added second PID controller to used measured rate for derivative instead of estimated error rate
    28 Apr 2016 - Added additional functions to allow gains to be changed on the fly    
    21 Aug 2016 - Added limit to control input to system

    Input/output variables: kp - proportional gain
                            kd - derivative gain
                            ki - integrator gain
                            I_L - integrator limit (absolute value)
                            target - setpoint of controller
                            actual - current system value
                            dadt - rate of change of the current system value
                            control_input_limit - [optional] controller input limit to system
               
    Author: Lars Soltmann
'''

import time

class PID:
    def __init__(self,kp,kd,ki,I_L):
        self.kp=kp
        self.kd=kd
        self.ki=ki
        self.I_L=I_L
        self.reset()

    # Reset PID controller components
    def reset(self):
        self.error_sum=0
        self.error_previous=0
        self.first_time=1

    # Set new PID gains
    def set_kp(self,new_kp):
        self.kp=new_kp

    def set_kd(self,new_kd):
        self.kd=new_kd

    def set_ki(self,new_ki):
        self.ki=new_ki

    # PID controller - derivative uses d(error)/dt
    def control(self,target, actual,control_input_limit='NONE'):
        if self.first_time==1:
            self.t_previous=time.time()
            control_input=0
            self.first_time=0
        else:
            # Get the current time and find differential time element
            t=time.time()
            dt=t-self.t_previous

            # Calculate current error
            error=target-actual
        
            # Estimate derivative of error
            _derivative=(error-self.error_previous)/dt;
        
            # Estimate integral of error
            self.error_sum=self.error_sum+error;
        
            # Apply integrator limits through error limiting
            if self.error_sum > self.I_L:
                self.error_sum=self.I_L
            if self.error_sum < -self.I_L:
                self.error_sum=-self.I_L
                
            _integral=self.error_sum

            # Calculate control input and limit if neccessary
            control_input=self.kp*error+self.kd*_derivative+self.ki*_integral
            if control_input_limit == 'NONE':
                pass
            else:
                if (control_input > control_input_limit):
                    control_input=control_input_limit
                elif (control_input < -control_input_limit):
                    control_input=-control_input_limit
                else:
                    pass        

            # Save time and error values for next loop;
            self.t_previous=t
            self.error_previous=error
    
        return control_input


    # PID controller - derivative uses measured d(actual)/dt
    def control2(self,target, actual, dadt,control_input_limit):
        if self.first_time==1:
            self.t_previous=time.time()
            control_input=0
            self.first_time=0
        else:
            # Get the current time and find differential time element
            t=time.time()
            dt=t-self.t_previous

            # Calculate current error
            error=target-actual
        
            # Set derivative as measured rate
            _derivative=dadt;
        
            # Estimate integral of error
            self.error_sum=self.error_sum+error;
        
            # Apply integrator limits through error limiting
            if self.error_sum > self.I_L:
                self.error_sum=self.I_L
            if self.error_sum < -self.I_L:
                self.error_sum=-self.I_L
                
            _integral=self.error_sum

            # Calculate control input and limit if neccessary
            control_input=self.kp*error+self.kd*_derivative+self.ki*_integral
            if control_input_limit == 'NONE':
                pass
            else:
                if (control_input > control_input_limit):
                    control_input=control_input_limit
                elif (control_input < -control_input_limit):
                    control_input=-control_input_limit
                else:
                    pass

            # Save time and error values for next loop;
            self.t_previous=t
            self.error_previous=error
    
        return control_input
