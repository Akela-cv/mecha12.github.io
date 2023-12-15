from pyb import Pin, Timer
class L6206:
    def __init__ (self,channel, PWM_tim, EFF_pin, DIR_pin, EN1_pin):
        self.enablepin = Pin(EN1_pin, mode=Pin.OUT_PP)
        self.EFF_pin = Pin(EFF_pin, mode=Pin.OUT_PP)
        self.DIR_pin = Pin(DIR_pin, mode=Pin.OUT_PP)
        self.pwm_ch_1 = PWM_tim.channel(channel, pin=EFF_pin, mode=Timer.PWM)
        

    def set_duty (self, duty):
        if duty >= 0:
            
            self.pwm_ch_1.pulse_width_percent(duty)
            self.DIR_pin.low()
        else: #Reverse
            duty = duty*-1
            self.pwm_ch_1.pulse_width_percent(duty)
            self.DIR_pin.high()
            
            
    def enable (self):
        self.enablepin.high()
        self.MotorOn = 1
        pass
        
    def disable (self):
        self.enablepin.low()
        self.MotorOn = 0
        pass
        