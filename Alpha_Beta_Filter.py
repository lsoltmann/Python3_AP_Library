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

    ## Track the state
    def track(self,xm,dt):
        # If it's the first time, set the current and previous states equal to the measurement
        if self.first_time==1:
            self.xk_1=xm
            self.vk_1=0
            xk=self.xk_1
            vk=self.vk_1
            self.first_time=0
    
        else:
            # Predit the next state while holding velocity constant
            # (It is assumed that the velocity is constant between corrections)
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
