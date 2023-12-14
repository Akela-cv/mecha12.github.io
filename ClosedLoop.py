
class ClosedLoop:
    
    def __init__(self):
        self.errsum = 0
        self.L = 0
        
    def OpenLoop(self, Setpoint, kp, ki, meas):
        pass
    
        
    def P_Control(self, Setpoint, kp, ki, meas):
        self.error = self.Setpoint - self.meas
        self.L = self.kp*self.error
        #Saturate
        if abs(self.L) >100:
            if self.L > 100:
                self.L = 100
            else:
                self.L = -100
        
        return(self.L)
        
    def PI_control(self, Setpoint, kp, ki, meas, errmax):
        self.error = Setpoint - meas
        self.errsum = self.errsum + self.error
        if abs(self.errsum>errmax):
            if self.errsum>0:
                self.errsum = errmax
            else:
                self.errsum = -errmax
        self.L = kp*self.error + ki*self.errsum
        #Saturate
        if abs(self.L) >100:
            if self.L>100:
                self.L = 100
            else:
                self.L = -100
            
        
        return(self.L)
        
    

