from gpiozero import DistanceSensor
from time import sleep

sensor_l = DistanceSensor(echo=1, trigger=7)
sensor_m = DistanceSensor(echo=9, trigger=10)
sensor_r = DistanceSensor(echo=26, trigger=16)
sensor_new_m = DistanceSensor(echo=15, trigger = 14)
while True:
    print('Distance_L: ', sensor_l.distance * 100)
    print('Distance_M: ', sensor_m.distance * 100)
    print('Distance_R: ', sensor_r.distance * 100)
    print('Distance_M_new: ', sensor_new_m.distance * 100)
    sleep(0.1)
    


#RIGHT ECHO=26, TRIG=16
#MID ECHO=9 TRIG=10
#LEFT ECHO=8 TRIG=7