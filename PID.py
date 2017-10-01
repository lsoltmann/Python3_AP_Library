'''
    PID.py
    
    Description: PID controller
    
    Revision History
    17 Mar 2016 - Created and debugged
    18 Apr 2016 - Updated, added second PID controller to used measured rate for derivative instead of estimated error rate
    28 Apr 2016 - Added additional functions to allow gains to be changed on the fly
    12 Apr 2017 - Refactored and added controller seeding and integrator freezing

    Author: Lars Soltmann
    
    References: None
    
    Calls: None
    
    Inputs: initialization      - kp,kd,ki = Proportional, derivative, integral gains
                                - I_L = integrator limit
                                
            seed_controller     - seed_value = user specified value to set the integrator term to
            
            freeze_integrator   - ON_OFF = turn on and off integrator freezing [0=freeze off, 1=freeze on]
            
            set_kp              - new_kp = new proportional gain
            
            set_kd              - new_kd = new derivative gain
            
            set_ki              - new_ki = new integral gain
            
            control             - target = setpoint of the controller
                                - actual = process variable
                                - type <defaults to 1> = determines whether to estimate the derivative of the error use a user specified rate [1=estimate d(error)/dt, 2=use a rate]
                                - dadt <defaults to 0> = user specified rate for derivative term
    
    Outputs: control            - controller_output = PID controller ouput
'''


import time

class PID:
    def __init__(self,kp,kd,ki,I_L):
        self.kp=kp
        self.kd=kd
        self.ki=ki
        self.I_L=I_L #Integrator limit, absolute value
        self.reset()

    # Reset PID controller components
    def reset(self):
        self.error_sum=0
        self.error_previous=0
        self.t_previous=0
        self.seed_flag=0
        self.freeze=0
        self.first_time=1
    
    # Seed controller with user specified integrator
    def seed_controller(self,seed_value):
        self.reset()
        self.I_TERM=seed_value
        self.seed_flag=1
    
    # Freeze the integrator, ON_OFF should be 1 for ON and 0 for OFF
    def freeze_integrator(self,ON_OFF):
        self.freeze=ON_OFF

    # Set new PID gains
    def set_kp(self,new_kp):
        self.kp=new_kp

    def set_kd(self,new_kd):
        self.kd=new_kd

    def set_ki(self,new_ki):
        self.ki=new_ki


    ## PID CONTROLLER
    # Type is either 1 or 2
    #   1 = estimate derivative of error for derivate term
    #   2 = use provided rate for derivative term
    def control(self, target, actual, type=1, dadt=0):
        if self.first_time==1:
            self.t_previous=time.time()
            if self.I_TERM!=0:
                controller_output=self.I_TERM
            else:
                controller_output=0
            self.first_time=0
        else:
            ## Get the current time and find differential time element
            t=time.time()
            dt=t-self.t_previous

            ## Calculate current error
            error=target-actual
            
            ## Caluclate P term
            P_TERM=error*self.kp
        
            ## Calculate derivative based on controller type
            if type==1:
                derivative_term=(error-self.error_previous)/dt
            elif type==2:
                derivative_term=dadt
            D_TERM=derivative_term*self.kd
        
            ## Estimate integral of error
            # If integral freeze is active, don't update error sum
            if self.freeze==1:
                pass
            # If integral freeze is inactive, proceed as normal
            else:
                self.error_sum=self.error_sum+error*dt

            # Calculate integral term if ki is not zero
            if self.ki != 0:
                # If the seed flag is set, use the specified integral term and calculate the associated error sum based on the current integral gain
                if self.seed_flag==1 and self.freeze==0:
                    self.error_sum=self.I_TERM/self.ki
                    self.seed_flag=0
                # If no seed flag is set, proceed as normal
                else:
                    self.I_TERM=self.error_sum*self.ki
            else:
                self.I_TERM=0
            
            '''
                # Apply integrator limits
                if self.I_TERM > self.I_L:
                self.I_TERM=self.I_L
                if self.I_TERM < -self.I_L:
                self.I_TERM=-self.I_L
            '''

            # Calculate controller output
            controller_output=P_TERM+D_TERM+self.I_TERM
        
            # Save time for next loop
            self.t_previous=t
            self.error_previous=error
    
        return controller_output
