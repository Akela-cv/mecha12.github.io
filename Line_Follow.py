from pyb import Timer, Pin, ADC, ExtInt
import time
from ClosedLoop import ClosedLoop
from encoder import Encoder 
from L6206 import L6206
from math import cos,sin,atan,sqrt
#Lab 3 Test
from micropython import alloc_emergency_exception_buf
alloc_emergency_exception_buf(1000)

class Follow_Class:
    def __init__(self):
        self.state = 0
        
    def Follow(self):
        global pos_x, pos_y,Heading,time_new,time_old
        def MotorSet(V_Setpoint,Omega_Setpoint):
            global pos_x, pos_y, Heading, time_new, time_old
            Setpoint_Motor_Right = (r*Omega_Setpoint - V_Setpoint)/r_wheel #Desired right wheel speed in rad/s
            Setpoint_Motor_Left = (-r*Omega_Setpoint - V_Setpoint)/r_wheel #Desired left wheel speed in rad/s
            
           
            #Convert from rad/s to increments/sec for motor control purposes
            
            time_new = time.ticks_ms()
            time_step = time.ticks_diff(time_new,time_old)
            time_old = time_new
            
            
            Right_Delta_Setpoint = Setpoint_Motor_Right*1/(2*3.1416)*1440*time_step/1000
            Left_Delta_Setpoint =  Setpoint_Motor_Left*1/(2*3.1416)*1440*time_step/1000
            
            #Update encoder readings and set motor control   
            
            ENC_A.update()
            ENC_B.update()
            #pos_A = ENC_A.get_position()
            #pos_B = ENC_B.get_position()
            delta_A = ENC_A.get_delta()
            delta_B = ENC_B.get_delta()
            duty_A = Control_Motor_A.PI_control(Left_Delta_Setpoint, kp_Motor, ki_Motor, delta_A, 0) 
            duty_B = Control_Motor_B.PI_control(Right_Delta_Setpoint, kp_Motor, ki_Motor, delta_B, 0) 
            
           
            #This prevents the motors from going too fast and making it difficult to stop romi from falling off the table
            if abs(duty_A)>60:
                if duty_A >0:
                    duty_A = 60 
                else:
                    duty_A = -60
            if abs(duty_B)>60:
                if duty_B >0:
                    duty_B = 60 
                else:
                    duty_B = -60
           
            mot_A.set_duty(-duty_A)
            mot_B.set_duty(-duty_B)
            
            #Using encoder data, calculate the velocity and angular velocity of the robot
            V_x = (-r_wheel*delta_A/1440*2*3.1416/time_step*1000 -r_wheel*delta_B/1440*2*3.1416/time_step*1000)*cos(Heading)/2
            V_y = (-r_wheel*delta_A/1440*2*3.1416/time_step*1000 -r_wheel*delta_B/1440*2*3.1416/time_step*1000)*sin(Heading)/2
            Omega_Heading = (-r_wheel*delta_B/1440*2*3.1416/time_step*1000+r_wheel*delta_A/1440*2*3.1416/time_step*1000)/(2*r)
            
            #Calculate absolute position and orientation based on position and delta data - Euler method integration
            
            #It remains to be seen how accurate this is
            pos_x = pos_x + V_x*time_step/1000
            pos_y = pos_y + V_y*time_step/1000
            Heading = Heading + Omega_Heading*time_step/1000 #This is incredibly inaccurate, need to use IMU instead
            
            
            #arc_length = (pos_A + pos_B)/2/1440*2*3.1416*r_wheel
            #arc_length = -arc_length

            #print(arc_length)
            #if arc_length>=.61*3.1416:
                #self.state = 2
                #print("Done")
        
        tim_A = Timer(1, freq = 20_000)
        tim_B = Timer(4, freq = 20_000)
        channel_A = 2
        channel_B = 2
        mot_A = L6206(channel_A,tim_A, Pin.cpu.A9, Pin.cpu.B2, Pin.cpu.B12) #LEFT SIDE
        mot_B = L6206(channel_B,tim_B, Pin.cpu.B7, Pin.cpu.H0, Pin.cpu.C15) #RIGHT SIDE
        
        tim_2 = Timer(2, period = 65535, prescaler = 0)    
        tim_3 = Timer(3, period = 65535, prescaler = 0)
        
        ENC_B = Encoder(tim_2, Pin.cpu.A0, Pin.cpu.A1) #RIGHT SIDE
        ENC_A = Encoder(tim_3, Pin.cpu.B4, Pin.cpu.B5) #LEFT SIDE
        
        ENC_A.zero()
        ENC_B.zero()
        
        
        mot_A.disable()
        mot_B.disable()
        mot_A.set_duty(0)
        mot_B.set_duty(0)
        
        start = time.ticks_ms()
        
        Line_Right_1 = ADC(Pin.board.PC2)
        Line_Right_2 = ADC(Pin.board.PC0)
        Line_Right_3 = ADC(Pin.board.PC3)
        Line_Center_1 = ADC(Pin.board.PC4)
        Line_Left_1 = ADC(Pin.board.PC1)
        Line_Left_2 = ADC(Pin.board.PB0)
        Line_Left_3 = ADC(Pin.board.PA4)
        
        #Error_Ratio = 10
        
        #Max motor speed no load: 150rpm
        V_max = .45 #Max speed for testing purposes, in m/s, absolute max .55
        V_inc = .02 #Velocity increase step
        V_dec = .05 #Velocity decrease step
        Omega_Max = 7 #Max angular velocity , absolute max 7.7
        
        kp_Line = .1
        ki_Line = 0
       
        kp_Motor = 10
        ki_Motor = 0
        Control_Omega = ClosedLoop()
        
        Calibration_Offset = 100 #Constant used to determine if we are going straight or not
        
        time_old = 0
        
        Control_Motor_A = ClosedLoop()
        Control_Motor_B = ClosedLoop()
        Setpoint_Left = 1000
        Setpoint_Right = 1000
        Center_High = 3100
        Center_Low = 2200
        Setpoint_Motor_Right = 0
        Setpoint_Motor_Left = 0
        V_Setpoint = .1 #Initial velocity setpoint in m/s
        
        r_wheel = .035 #Wheel radius in m
        r = .073 #Romi radius in m
        
        Obstacle_Length = 8*2.54/100 #Obstacle length in m
        
        ButtonPin = Pin(Pin.cpu.C13, mode=Pin.IN)
        
        Done_flag = 0
        
        
        while True:
            try:
                if self.state == 0:
                    if time.ticks_diff(time.ticks_ms(),start) <1000:
                        pass
                        
                    else:
                        mot_A.enable()
                        mot_B.enable() #This does not work
                        mot_A.set_duty(0)
                        mot_B.set_duty(0)
                        ENC_A.zero()
                        ENC_B.zero()
                        ENC_A.update()
                        ENC_B.update()
                        posA = ENC_A.get_position()
                        
                        #Set position to zero
                        pos_x = 0
                        pos_y = 0
                        Heading = 0
                        
                        #Read initial IMU value
                       
                        
                        #Get button input, start line follow if true 
                        if ButtonPin.value():
                            self.state=0
                           
                        else:
                            self.state = 1
                            start = time.ticks_ms()
                       
                        
                        
                if self.state == 1: #Read and update controller
                    
                    
                    
                    #Read analog values
                    
                    Right_Analog1 = Line_Right_1.read() #Closest to Center
                    #Right_Analog2 = Line_Right_2.read()
                    Right_Analog3 = Line_Right_3.read() #Furthest from Center
                    
                   
                    Center_Analog1 = Line_Center_1.read()
                    Left_Analog1 = Line_Left_1.read() #Closest to Center
                    #Left_Analog2 = Line_Left_2.read()
                    Left_Analog3 = Line_Left_3.read() #Furthest from Center
                
                    #Combined_Right = (Right_Analog1 + Right_Analog1 + Right_Analog1)/3
                    #Combined_Left = (Left_Analog1 + Left_Analog1 + Left_Analog1)/3
                    
                 
                   
                    #Using read values, find the difference between the left and right side to determine whether to turn right or left
                    
                    Diff = Left_Analog1-Right_Analog1 # + to drift Left
                    Diff2 = Left_Analog3-Right_Analog3
                    
                    #If you see a difference in the inner ones, then turn. If not check the outer sensors. If still no, go straight
                    if abs(Diff)>Calibration_Offset:
                        Duty_Omega = Control_Omega.PI_control(0, kp_Line, ki_Line, Diff, 0)
                    elif abs(Diff2)>Calibration_Offset*2:
                        Duty_Omega = Control_Omega.PI_control(0, kp_Line, ki_Line, Diff2, 0)
                    else:
                        Duty_Omega = 0
                    
                    Omega_Setpoint_1 = Duty_Omega/100*Omega_Max
                    
                    #Using center value, determine if to speed up or slow down, and saturate to Vmax or 0
                    
                    #Instead of speeding up or slowing down, having two or more velocity settings based on center reading may be better
                    
                    V_Setpoint_1 = V_max
                    
                    MotorSet(V_Setpoint_1,Omega_Setpoint_1)
                   
                            
                    #Check if my outside readings were both high 
                    if Right_Analog3>2000 and Left_Analog3>2000 and Done_flag == 0:
                        pos_detectx = pos_x
                        pos_detecty = pos_y
                        
                        Done_flag = 1
                    if Done_flag == 1:   
                        pos_dif = sqrt(pow(pos_x-pos_detectx,2)+pow(pos_y-pos_detecty,2))
                        print(pos_dif)
                        if pos_dif>0.250 and pos_dif<0.270:
                            if Right_Analog3>2000 and Left_Analog3>2000:
                                self.state = 0 #Set this to state 3 later
                                Done_flag = 0
                                print(1)
                                
                        elif pos_dif>.270:
                            Done_flag = 0
                            
                            
                        
                    
                    
                   
                        
                    #Checking if button is pressed, if it is, return to initialization state   
                    if ButtonPin.value():
                        pass
                               
                    else:
                        if time.ticks_diff(time.ticks_ms(),start) <2000:
                            pass
                        else:
                            self.state = 0
                            start = time.ticks_ms()
                            mot_A.set_duty(0)
                            mot_B.set_duty(0)
                        
                if self.state == 2: #Go around the obstacle
                    #Checking if button is pressed, if it is, return to initialization state   
                    if ButtonPin.value():
                        self.state=1
                    else:
                        if time.ticks_diff(time.ticks_ms(),start) <2000:
                            pass
                        else:
                              self.state = 0
                              start = time.ticks_ms()
                              mot_A.set_duty(0)
                              mot_B.set_duty(0)
                              
                              
                    #Step 1, turn 90 degrees, using IMU reading
                    #Step 2, drive straght legnth, based on position readings
                    #Step 3, turn 90 degrees, using IMU reading
                    #Step 4: check if obstacle still there, if yes return to step 1, if no drive straight 1 length
                    #Step 5: turn 90 dgerees, 
                    
                    
                    
                if self.state == 3: #Stop, then return to start  
                    
                    #Checking if button is pressed, if it is, return to initialization state   
                    if ButtonPin.value():
                        self.state=1
                    else:
                        if time.ticks_diff(time.ticks_ms(),start) <2000:
                            pass
                        else:
                            self.state = 0
                            start = time.ticks_ms()
                            mot_A.set_duty(0)
                            mot_B.set_duty(0)
                
                
                    if time.ticks_diff(time.ticks_ms(),start) <1000:
                       mot_A.disable()
                       mot_B.disable()
                       mot_A.set_duty(0)
                       mot_B.set_duty(0) 
                       time_old = 0
                       #Calculate offset vector 
                       pos_startx = pos_x
                       pos_starty = pos_y
                       
                       theta_vector = atan(-pos_starty/-pos_startx)
                       #Code determining the actual heading based on the value of the posiiton
                       if pos_startx>0 and pos_starty>0:
                           theta_vector = theta_vector + 3.14159 #offset by pi
                       if pos_startx>0 and pos_starty<0:
                           theta_vector = theta_vector + 3.14159 #offset by pi
                       if pos_startx<0 and pos_starty>0:
                           theta_vector = theta_vector+6.283185 #offset by 2pi
                       if pos_startx<0 and pos_starty<0:
                           theta_vector = theta_vector #No offset needed
                       
                       theta_start = 1 #Get IMU value
                       
                       theta_target = theta_vector-theta_start #Target heading
                       
                       
                    else:
                        #Get reading from IMU, run one motor forward and other in reverse until reach desired heading
                        theta_current = 1 #IMU reading
                        #May need to get Imu reading and control if we cant drive straight
                        MotorSet(0,1.5)
                        
                        if abs(theta_current-theta_target)<.05: #If we have reached our target heading
                            if False: #if we see something in the way, then go to state 2
                                self.state = 2
                                #IF: Insert code here to go to obstacle state, then recalculate heading
                                
                            elif abs(pos_x)<1: #If we returned to our start position, then stop
                                self.state = 1
                            
                            else: #Otherwise keep driving straight
                                
                                MotorSet(V_max,0)
            
            
            except KeyboardInterrupt:
                mot_A.disable()
                mot_B.set_duty(0)
                mot_B.disable()
                break