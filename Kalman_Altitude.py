'''
    Kalman_Altitude.py
    
    Description: Kalman filter for altitude determination 
                 using 3 distance/height measurements and 
                 1 vertical velocity measurement
                 
    Inputs: init
                - Process noise (q)
                - Measurement noise (r)
                - Covariance (p)
                - States (x)
            alt_kf
                - Measurements (z)
                - Observation (h)
                - Time increment (dt)
                
            * See Notes for input format

    Notation: 1 - Range finder
	      2 - Barometer
	      3 - GPS alt
              4 - GPS vert vel
    
    Revision History
    17 Aug 2016 - Created and debugged
    
    Author: Lars Soltmann
    
    Notes:
    - Written for Python3
    - Originally designed for use with any combination of
      ultrasonic rangefinder, barometer, and GPS (h and
      h_dot)
    - Input format for any matrix is done with a list and
      will follow the format: row 1 all columns, 
      row 2 all columns, row 3 all columns, ...
        example for 3x3 matrix
            x = [M(1,1),M(1,2),M(1,3),M(2,1),M(2,2),M(2,3),M(3,1),M(3,2),M(3,3)]
            
    References: - Kalman_Altitude_equations.mw (MAPLE 2016 file)
        
    
'''

class alt_kalman:
    def __init__(self,p,q,r,x):
        ## Initialize covariance matrix
        self.p1=p[0]
        self.p2=p[1]
        self.p3=p[2]
        self.p4=p[3]
        
        ## Initialize process noise matrix
        self.q1=q[0]
        self.q2=q[1]
        
        ## Initialize measurement noise matrix
        self.r1=r[0]
        self.r2=r[1]
        self.r3=r[2]
        self.r4=r[3]
        
        ## Initialize state matrix
        self.x1=x[0]
        self.x2=x[1]


    def alt_kf(self,h,z,dt):
        ## Set observation matrix
        h1=h[0]
        h2=h[1]
        h3=h[2]
        h4=h[3]
        
        z1=z[0]
        z2=z[1]
        z3=z[2]
        z4=z[3]
        
        ## NOTE: the below sections were generated using code optimization in MAPLE 2016 to minimize function calls
        ## Create temporary variables to reduce overall number of calls while creating Kalman gain matrix
        t1=h2**2
        t2=t1*self.p1
        t3=h3**2
        t4=t3*self.r2
        t5=t4*self.p1
        t6=(self.r2+t2)*self.r3+t5
        t7=h4**2
        t8=t1*self.r3
        t9=t7*self.p4
        t6=self.r4*t6+t9*t6-t7*(t4+t8)*self.p3*self.p2
        t10=h1**2
        t11=self.p1*t10+self.r1
        t2=t2*self.r1+self.r2*t11
        t5=t5*self.r1+self.r3*t2
        t12=t10*self.r2
        t1=t1*self.r1
        t13=-t1-t12
        t14=t4*self.r1
        t15=self.r4*t5+t9*t5+t7*(self.r3*t13-t14)*self.p3*self.p2
        t7=t7*self.p2*self.p3
        t16=(-self.r4-t9)*self.p1+t7
        t15=0.1e1/t15
        t4=(t4+t8)*t16
        t8=t7*self.r3
        t17=t15*h1
        t3=t3*self.r1
        t11=t3*self.p1+self.r3*t11
        t10=t10*self.r3
        t11=self.r4*t11+t9*t11-t7*(t10+t3)
        t3=(t10+t3)*t16
        t10=t15*h2
        t2=self.r4*t2+t7*t13+t9*t2
        t13=(t1+t12)*t16
        t16=t15*h3
        t1=(-t1-t12)*self.r3-t14
        t12=t9*self.r2
        
        ## Assemble Kalman gain matrix components
        ksim1=t17*((t4+t6)*self.p1-t8*self.r2)
        ksim2=t10*((t3+t11)*self.p1-t8*self.r1)
        ksim3=t16*((t13+t2)*self.p1-t7*self.r1*self.r2)
        ksim4=t15*self.p2*h4*(t1*self.p1+t5)
        ksim5=t17*self.p3*(-t12*self.r3+t4+t6)
        ksim6=t10*self.p3*(-t9*self.r1*self.r3+t11+t3)
        ksim7=t16*self.p3*(-t12*self.r1+t13+t2)
        ksim8=t15*h4*(self.p3*self.p2*t1+self.p4*t5)
        
        ## Estimate states
        xest1=self.x1+ksim1*(-h1*self.x1+z1)+ksim2*(-h2*self.x1+z2)+ksim3*(-h3*self.x1+z3)+ksim4*(-h4*self.x2+z4)
        xest2=self.x2+ksim5*(-h1*self.x1+z1)+ksim6*(-h2*self.x1+z2)+ksim7*(-h3*self.x1+z3)+ksim8*(-h4*self.x2+z4)

        ## Estimate covariance matrix components
        pest1=(-h1*ksim1-h2*ksim2-h3*ksim3+1)*self.p1-ksim4*h4*self.p3
        pest2=(-h1*ksim1-h2*ksim2-h3*ksim3+1)*self.p2-ksim4*h4*self.p4
        pest3=(-h1*ksim5-h2*ksim6-h3*ksim7)*self.p1+(-h4*ksim8+1)*self.p3
        pest4=(-h1*ksim5-h2*ksim6-h3*ksim7)*self.p2+(-h4*ksim8+1)*self.p4

        ## Create temporary variables for state prediction
        t1=-h1*self.x1+z1
        t2=-h2*self.x1+z2
        t3=-h3*self.x1+z3
        t4=-h4*self.x2+z4
        t5=ksim5*t1+ksim6*t2+ksim7*t3+ksim8*t4+self.x2
        
        ## Predict the next state
        self.xpred1=dt*t5+ksim1*t1+ksim2*t2+ksim3*t3+ksim4*t4+self.x1
        self.xpred2=t5

        ## Create temporary variables for covariance prediction
        t1=-h1*ksim1-h2*ksim2-h3*ksim3+1
        t2=h1*ksim5+h2*ksim6+h3*ksim7
        t3=-h4*ksim8+1
        t4=t2*self.p1
        t5=t3*self.p3
        t2=t2*self.p2
        t3=t3*self.p4
        t6=dt*(t2-t3)
        t7=ksim4*h4
        t8=-self.p2*t1+t7*self.p4+t6
        
        ## Predict next covariance matrix
        self.ppred1=-(t4-t5+t8)*dt+self.p1*t1+self.q1-t7*self.p3
        self.ppred2=-t8
        self.ppred3=-t6-t4+t5
        self.ppred4=self.q2-t2+t3

        ## Save states and covariance for the next iteration
        self.x1=self.xpred1
        self.x2=self.xpred2
        self.p1=self.ppred1
        self.p2=self.ppred2
        self.p3=self.ppred3
        self.p4=self.ppred4

        return [xest1,xest2]
