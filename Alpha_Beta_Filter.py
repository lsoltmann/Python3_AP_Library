'''
    Alpha_Beta_Filter.py
    
    Description: Alpha-beta filter used for state tracking
    
    Revision History
    29 May 2017 - Created and debugged
    
    Author: Lars Soltmann
    
    References: - https://en.wikipedia.org/wiki/Alpha_beta_filter
    
    Notes:
    - Written for Python3
    
    Requirements: None
    
'''

class trackfilt:
    def __init__(self,alpha,beta):
        self.alpha=alpha
        self.beta=beta
        self.N_avg=5 #Number of samples to average after a reset to start off the filter
        self.reset()

    ## Set new alpha coefficient
    def set_alpha(self,newalpha):
        self.alpha=newalpha

    ## Set new beta coefficient
    def set_beta(self,newbeta):
        self.beta=newbeta

    ## Reset filter
    def reset(self):
        self.first_time=1
        self.avg_count=1
        self.xk_1=0
        self.vk_1=0

    ## Track the state
    def track(self,xm,dt):
        # If it's the first time, set the previous state equal to an average of the current state
        if self.first_time==1:
            self.xk_1=self.xk_1+xm/self.N_avg
            if self.avg_count==self.N_avg:
                self.first_time=0 #Exit 'first_time' loop when target number of samples have been averaged
            else:
                self.avg_count=self.avg_count+1
            xk=self.xk_1
            vk=self.vk_1
        else:
            # Predit the next state while holding velocity constant (generally acceptable for small time increments)
            xk=self.xk_1+(self.vk_1*dt)
            vk=self.vk_1
            
            # Get prediction error
            rk=xm-xk
            
            # Correct the state estimates using the defined filter coefficients and prediction error
            xk=xk+self.alpha*rk
            vk=vk+(self.beta*rk)/dt
            
            # Save the states for the next loop
            self.xk_1=xk
            self.vk_1=vk

        return xk,vk
