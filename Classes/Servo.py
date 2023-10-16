from CONFIG import *
from gpiozero import AngularServo
from time import sleep

class Servo:
    def __init__(self, pin):
        self.servo = AngularServo(pin,min_angle=0,max_angle=10, initial_angle=0)
    def setup(self):
        self.servo.min()
    def dropoff(self):
        self.servo.max()
        sleep(2)
        self.servo.min()