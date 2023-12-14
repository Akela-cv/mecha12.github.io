import cotask
import task_share
from Line_Follow import Follow_Class
import time
from pyb import Pin
# trig_pin = Pin(Pin.cpu.B13, mode=Pin.OUT)
# echo_pin = Pin(Pin.cpu.B14, mode = Pin.IN)
# pulse_width = 0
# for n in range(50000):
#     trig_pin.high()
#     time.sleep_us(10)
#     trig_pin.low()

#     start = time.ticks_ms
#     print(echo_pin.value())
#     if((echo_pin.value()) == 1):
#         print('hi')
#         pulse_width = time.ticks_diff(time.ticks_ms(),start)
#         distance = pulse_width / 58.0
#         print(distance)

#     time.sleep_ms(60)
    




fun1 = Follow_Class()

task1 = cotask.Task (fun1.Follow(), name = 'Task 1', priority = 1, 
                      period = 100, profile = True, trace = True)

 
# Add the task to the list (so it will be run) and run scheduler
cotask.task_list.append (task1)


while True: 
    try:
        cotask.task_list.pri_sched()
    except KeyboardInterrupt:
        break
    

