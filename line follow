from pyb import Timer, Pin, ADC, ExtInt
import time
from ClosedLoop import ClosedLoop
from encoder import Encoder 
from L6206 import L6206
from math import cos,sin,atan,sqrt
from hcsr04 import HCSR04
from IMU import imu
from micropython import alloc_emergency_exception_buf
alloc_emergency_exception_buf(1000)

class Follow_Class:
    def __init__(self):
        self.state = 0
        
    def Follow(self):
        """
        Main state machine for the line-following robot.

        The function performs line-following using PI control and line sensor readings, obstacle avoidance using an ultrasonic sensor, 
        and navigation back to the  starting point when the finish line has been detected. 
        Encoder readings and IMU readings are used to kinematically model the position and heading of the robot over time.  

        """
        global pos_x, pos_y,Heading,time_new,time_old
       
        def MotorSet(V_Setpoint,Omega_Setpoint):
            """
            Set motor velocities based on velocity and angular velocity setpoints.

            Args:
                V_Setpoint (float): Velocity setpoint in m/s.
                Omega_Setpoint (float): Angular velocity setpoint in rad/s.
            """
            
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
            Heading = IMU_1.Euler_Angle()/900-Heading_1
            
            
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
        
        
        #Iniialize HCSR04, using code found from internet
        US_Sensor = HCSR04(Pin.cpu.B13,Pin.cpu.B14)
        wall_dist = 7 #Distance from wall measured, in cm
        
        #Initialize IMU
        IMU_1 = imu()
        
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
        V_max = .3 #Max speed for testing purposes, in m/s, absolute max .55, nominal .35
        V_inc = .02 #Velocity increase step
        V_dec = .05 #Velocity decrease step
        Omega_Max = 6 #Max angular velocity , absolute max 7.7, nominal 6
        turn_rate = 3 #Turning rate used for 90 degree turns
        
        kp_Line = .07
        ki_Line = .00001
       
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
        
        
        
        
        while True:
            try:
                print(US_Sensor.distance_cm())
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
                        
                        #Flags:
                        Done_flag = 0
                        Finsihed = 0
                        Turn_Flag_1 = 0
                        Turn_Flag_2 = 0
                        Turn_Flag_3 = 0
                        Turn_Flag_4 = 0
                        Turn_Flag_5 = 0
                        Turn_Flag_6 = 0
                        Turn_Flag_7 = 0
                        Turn_Flag_8 = 0
                        Final_Turn = 0
                        
                        #Set position to zero
                        pos_x = 0
                        pos_y = 0
                        Heading = 0
                        
                        #Read initial IMU value
                        Heading_1 = IMU_1.Euler_Angle()/900
                        
                       
                        
                        #Get button input, start line follow if true 
                        if ButtonPin.value():
                            self.state=0
                           
                        else:
                            self.state = 1
                            start = time.ticks_ms()
                       
                        
                        
                if self.state == 1: #Follow the line
                    
                    #Read analog values
                    Right_Analog1 = Line_Right_1.read() #Closest to Center
                    #Right_Analog2 = Line_Right_2.read()
                    Right_Analog3 = Line_Right_3.read() #Furthest from Center
                    
                   
                    Center_Analog1 = Line_Center_1.read()
                    Left_Analog1 = Line_Left_1.read() #Closest to Center
                    #Left_Analog2 = Line_Left_2.read()
                    Left_Analog3 = Line_Left_3.read() #Furthest from Center
                
                    #Using read values, find the difference between the left and right side to determine whether to turn right or left
                    Diff = Left_Analog1-Right_Analog1 # + to drift Left
                    Diff2 = Left_Analog3-Right_Analog3
                    
                    #If significant difference in the inner sensors, then turn. If not check the outer sensors. If still no, go straight
                    if abs(Diff)>Calibration_Offset:
                        Duty_Omega = Control_Omega.PI_control(0, kp_Line, ki_Line, Diff, 500)
                    elif abs(Diff2)>Calibration_Offset*2:
                        Duty_Omega = Control_Omega.PI_control(0, kp_Line, ki_Line, Diff2, 200)
                    else:
                        Duty_Omega = 0
                    
                    #Set angular velocity target based on max
                    Omega_Setpoint_1 = Duty_Omega/100*Omega_Max
                    
                    #Set velocity and run motors accordingly
                    V_Setpoint_1 = V_max
                    MotorSet(V_Setpoint_1,Omega_Setpoint_1)
                   
                            
                    #Check if theres a wall in front, if yes then go to state 2
                    if US_Sensor.distance_cm() < wall_dist and US_Sensor.distance_cm()>.1:
                        self.state = 2 #Make next state 2
                        #Stop the robot
                        Turn_Flag_1 = 1 #Set turn flag
                        Turn_Start = Heading #Set initial turn value
                        mot_A.set_duty(0) 
                        mot_B.set_duty(0)
                    
                    
                    #Check if my outside readings were both high, indicating potential finish 
                    if Right_Analog3>2000 and Left_Analog3>2000 and Done_flag == 0:
                        pos_detectx = pos_x
                        pos_detecty = pos_y
                        Done_flag = 1 #Set flag indicating potential finish
                    
                    if Done_flag == 1:  #Calcualte distance traveled 
                        pos_dif = sqrt(pow(pos_x-pos_detectx,2)+pow(pos_y-pos_detecty,2))
                        
                        if pos_dif>0.250 and pos_dif<0.270: #If distance traveled lines up with size of finish
                            #Check if black line detected, and if so go to state 3
                            if Right_Analog3>2000 and Left_Analog3>2000: 
                                self.state = 3 
                                Done_flag = 0
                                Finished = 1
                                start = time.ticks_ms()
                                
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
                        self.state=2
                    else:
                        if time.ticks_diff(time.ticks_ms(),start) <2000:
                            pass
                        else:
                              self.state = 0
                              start = time.ticks_ms()
                              mot_A.set_duty(0)
                              mot_B.set_duty(0)
                              
                              
                    #Step 1, turn 90 degrees, using IMU reading
                    if Turn_Flag_1 == 1:
                        MotorSet(0,turn_rate) #Turn left
                        if abs(Heading-Turn_Start)>=3.14159/2: #If we have turned 90 degrees
                            Turn_Flag_1 = 0
                            mot_A.set_duty(0)
                            mot_B.set_duty(0)
                            Turn_Flag_2 = 1
                            turn_posx1 = pos_x #Set initial position
                            turn_posy1 = pos_y #Set initial position
                            
                    #Step 2, drive straght length, based on position readings
                    if Turn_Flag_2 == 1:
                        MotorSet(V_max,0) #Drive straight
                        pos_dif = sqrt(pow(pos_x-turn_posx1,2)+pow(pos_y-turn_posy1,2))
                        if pos_dif>=Obstacle_Length*1.2:
                            Turn_Flag_2 = 0
                            mot_A.set_duty(0)
                            mot_B.set_duty(0)
                            Turn_Flag_3 = 1
                            Turn_Start = Heading #Set initial turn value
                    #Step 3, turn right 90 degrees, using IMU reading
                    if Turn_Flag_3 == 1:
                        MotorSet(0,-turn_rate) #Turn right
                        if abs(Heading-Turn_Start)>=3.14159/2: #If we have turned 90 degrees
                            Turn_Flag_3 = 0
                            mot_A.set_duty(0)
                            mot_B.set_duty(0)
                            Turn_Flag_4 = 1
                            turn_posx1 = pos_x #Set initial position
                            turn_posy1 = pos_y #Set initial position
                        
                    #Step 4, drive straight length of obstacle*2
                    if Turn_Flag_4 == 1:
                        MotorSet(V_max,0)
                        pos_dif = sqrt(pow(pos_x-turn_posx1,2)+pow(pos_y-turn_posy1,2))
                        if pos_dif>=(Obstacle_Length*2.75):
                            Turn_Flag_4 = 0
                            mot_A.set_duty(0)
                            mot_B.set_duty(0)
                            Turn_Flag_5 = 1
                            Turn_Start = Heading #Set initial turn value 
                    
                    #Step 5, turn right 90 degrees
                    if Turn_Flag_5 == 1:
                        MotorSet(0,-turn_rate) #Turn right
                        if abs(Heading-Turn_Start)>=3.14159/2: #If we have turned 90 degrees
                            Turn_Flag_5 = 0
                            mot_A.set_duty(0)
                            mot_B.set_duty(0)
                            Turn_Flag_6 = 1
                            turn_pos1 = pos_x #Set initial position  
                    
                    #Step 6: check if obstacle still there, if yes return to step 1, if no drive straight until you see a line
                    if Turn_Flag_6 == 1:
                        if US_Sensor.distance_cm() < wall_dist*2: #If we see a wall, return to step 1
                            Turn_Flag_6 = 0
                            Turn_Flag_1 = 1
                            Turn_Start = Heading #Set initial turn value
                            mot_A.set_duty(0) 
                            mot_B.set_duty(0)
                        else: #Otherwise, drive straight until see a line
                            MotorSet(V_max,0)
                            Right_Analog3 = Line_Right_3.read() #Furthest from Center
                            Left_Analog3 = Line_Left_3.read() #Furthest from Center
                            if Right_Analog3>2000: #If see line, go to next step
                                Turn_Flag_6 = 0
                                Turn_Flag_7 = 1
                                turn_posx1 = pos_x #Set initial position
                                turn_posy1 = pos_y #Set initial position
                                mot_A.set_duty(0) 
                                mot_B.set_duty(0)
                    
                    #Step 7, drive straight length of robot
                    if Turn_Flag_7 == 1:
                        MotorSet(V_max,0)
                        pos_dif = sqrt(pow(pos_x-turn_posx1,2)+pow(pos_y-turn_posy1,2))
                        if pos_dif>=(.075):
                            Turn_Flag_7 = 0
                            mot_A.set_duty(0)
                            mot_B.set_duty(0)
                            Turn_Flag_8 = 1
                            Turn_Start = Heading #Set initial turn value 
                    
                    
                    #Step 8: Drive straight .08m, then turn 90 degrees, then go back to following line
                    if Turn_Flag_8 == 1:
                        MotorSet(0,turn_rate) #Turn left
                        if abs(Heading-Turn_Start)>=3.14159/2: #If we have turned 90 degrees
                            Turn_Flag_8 = 0
                            mot_A.set_duty(0)
                            mot_B.set_duty(0)
                            if Finished == 1: #If we finished, go to state 3, otherwise keep following line
                                self.state = 3
                            else:
                                self.state = 1
                            
                if self.state == 3: #Stop, then return to start  
                    
                    #Checking if button is pressed, if it is, return to initialization state   
                    if ButtonPin.value():
                        self.state=3
                    else:
                        if time.ticks_diff(time.ticks_ms(),start) <2000:
                            pass
                        else:
                            self.state = 0
                            start = time.ticks_ms()
                            mot_A.set_duty(0)
                            mot_B.set_duty(0)
                
                
                    if time.ticks_diff(time.ticks_ms(),start)<250:
                       mot_A.set_duty(0)
                       mot_B.set_duty(0) 
                       time_old = 0
                       #Calculate offset vector 
                       pos_startx = pos_x
                       pos_starty = pos_y
                       
                       Final_turn = 1
                       
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
                       
                       theta_start = Heading #Get IMU value
                       
                       theta_target = theta_vector-theta_start #Target heading
                       
                    else: 
                        
                        theta_current = Heading #IMU reading
                        if abs(theta_current-theta_target)<=.15 or abs(6.283185+theta_current-theta_target)<=.15: #If we have reached our target heading
                            # if US_Sensor.distance_cm() < wall_dist: #if we see something in the way, then go to state 2
                            #     self.state = 2
                            #     Turn_Flag_1 = 1 #Set turn flag
                            #     Turn_Start = Heading #Set initial turn value
                            #     mot_A.set_duty(0) 
                            #     mot_B.set_duty(0)
                                
                            if abs(pos_x)<.01: #If we returned to our start position, then stop
                                self.state = 0
                            
                            else: #Otherwise keep driving straight
                                MotorSet(V_max,0)
                        else:
                            delta = theta_target-theta_current
                            if (delta)>0 and abs(delta)<3.14159:
                                MotorSet(0,turn_rate/2) #Turn left
                            if delta<0 and abs(delta)>3.14159:
                                MotorSet(0,turn_rate/2) #Turn left
                            if (delta)>0 and abs(delta)>3.14159:
                                MotorSet(0,-turn_rate/2) #Turn right
                            if delta<0 and abs(delta)<3.14159:
                                MotorSet(0,-turn_rate/2) #Turn right
                            
            except KeyboardInterrupt:
                mot_A.disable()
                mot_B.set_duty(0)
                mot_B.disable()
                break