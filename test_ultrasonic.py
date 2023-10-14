from gpiozero import DistanceSensor
from time import sleep
from CONFIG import *

class Ultra:
    counter_max = 10
    def __init__(self, echo, trigger) -> None:
        self.sensor = DistanceSensor(echo, trigger)
        self.old_val = 0
        self.counter = 0

    def distance(self):
        val = self.sensor.distance
        if val == self.old_val and self.counter >= Ultra.counter_max:
            return 1
        elif val == self.old_val:
            self.counter += 1
        else:
            self.old_val = val
            self.counter = 0
        return val

#backleft 
sensor_F = DistanceSensor(echo=ULT_FRONT_ECHO, trigger=ULT_FRONT_TRIG)
#frontleft 
#sensor_SF = DistanceSensor(echo=ULT_SIDEFRONT_ECHO, trigger=ULT_SIDEFRONT_TRIG)
#front one
#sensor_SB = DistanceSensor(echo=ULT_SIDEBACK_ECHO, trigger=ULT_SIDEBACK_TRIG)

#sensor_new_m = Ultra(echo=26, trigger = 16)
while True:
    print('Distance_F: ', sensor_F.distance * 100)
    #print('Distance_SF: ', sensor_SF.distance * 100)
    #print('Distance_SB: ', sensor_SB.distance * 100)
    #print('Distance_M_new: ', sensor_new_m.distance() * 100)
    sleep(0.1)
    




      
   

#RIGHT ECHO=26, TRIG=16
#MID ECHO=9 TRIG=10
#LEFT ECHO=8 TRIG=7
