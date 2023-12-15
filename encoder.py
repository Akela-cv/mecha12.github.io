from pyb import Timer
class Encoder:
    """The Encoder class interfaces with the encoder that is encased within the motor.

    """

    def __init__(self,ENC_tim,CH_A_PIN,CH_B_PIN):
          """__init__ method for the encoder.

        Args:
            ENC_tim (str): The timer channel of the encoder.
            CH_A_PIN (str): The pin object of the encoder for motor A
            CH_B_PIN (str): The pin object of the encoder for motor B
        """
        self.ench1 = ENC_tim.channel(1, pin=CH_A_PIN, mode=Timer.ENC_AB)
        self.ench2 = ENC_tim.channel(2, pin=CH_B_PIN, mode=Timer.ENC_AB)
        self.Timer = ENC_tim       
        pass
    
    def update(self):       
         """Updates the current position of the encoder"""
        self.count_new = self.Timer.counter()
        self.delta = self.count_new-self.count_old
        
        if abs(self.delta)<32768:
            self.pos = self.pos + self.delta
            self.count_old = self.count_new
            
        elif self.delta<0: #Saturate
            self.count_new = self.count_new + 65536
            self.delta = self.count_new-self.count_old
            self.pos = self.pos + self.delta
            self.count_old=self.count_new - 65536
            
        elif self.delta>0:  #Saturate
            self.count_new = self.count_new - 65536
            self.delta = self.count_new-self.count_old 
            self.pos = self.pos + self.delta
            self.count_old=self.count_new + 65536
        else:
            print ("No")  
        pass
    
    def get_position(self):
        self.pos = self.pos     
        return self.pos
    
    
    def get_delta(self):
        self.delta = self.delta
        return self.delta
    
    def zero(self):
         """str: Zeros the encoder"""
        self.pos = 0
        self.count_old = 0
        self.count_new = 0
        self.count = 0
        self.delta = 0
        
        pass
