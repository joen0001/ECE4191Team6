from CONFIG import *  
from gpiozero import DistanceSensor

def Ultrasonic():
  sensor_front = DistanceSensor(ULT_FRONT_ECHO, ULT_FRONT_TRIG)
  sensor_right = DistanceSensor(ULT_RIGHT_ECHO, ULT_RIGHT_TRIG)
  sensor_left = DistanceSensor(ULT_LEFT_ECHO, ULT_LEFT_TRIG)
  return sensor_front.distance*100, sensor_right.distance*100, sensor_left.distance*100
