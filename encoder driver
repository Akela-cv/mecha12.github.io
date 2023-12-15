from pyb import Timer
class Encoder:
    
    def __init__(self,ENC_tim,CH_A_PIN,CH_B_PIN):
        
        self.ench1 = ENC_tim.channel(1, pin=CH_A_PIN, mode=Timer.ENC_AB)
        self.ench2 = ENC_tim.channel(2, pin=CH_B_PIN, mode=Timer.ENC_AB)
        self.Timer = ENC_tim       
        pass
    
    def update(self):
        
        self.count_new = self.Timer.counter()
        
        self.delta = self.count_new-self.count_old
        
        if abs(self.delta)<32768:
            self.pos = self.pos + self.delta
            self.count_old = self.count_new
            
        elif self.delta<0:
            #Saturate
            self.count_new = self.count_new + 65536
            self.delta = self.count_new-self.count_old
            self.pos = self.pos + self.delta
            self.count_old=self.count_new - 65536
            
        elif self.delta>0: 
            #Saturate
            #print("Test")
            self.count_new = self.count_new - 65536
            self.delta = self.count_new-self.count_old 
            self.pos = self.pos + self.delta
            self.count_old=self.count_new + 65536
        else:
            print ("No")  
        pass
    
    
    def get_position(self):
        
        #print(pos)
        self.pos = self.pos
        
        return self.pos
    
    
    def get_delta(self):
        
        #print(delta)
        self.delta = self.delta
        
        return self.delta
    
    
    def zero(self):
        
        
        self.pos = 0
        self.count_old = 0
        self.count_new = 0
        self.count = 0
        self.delta = 0
        
        pass
