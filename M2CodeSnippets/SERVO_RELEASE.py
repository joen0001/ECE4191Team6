location = [0,0,0]
destination = [0,0,0]

#SETUP

from CONFIG import *
from gpiozero import Servo
from time import sleep

servo = Servo(SERVO_PIN)

# FOR MAIN LOOP

x_dist = abs(destination[0] - location[0])
y_dist = abs(destination[1] - location[1])
theta_diff = abs(destination[2] - location[2])

if x_dist < DIST_THRESHOLD and y_dist < DIST_THRESHOLD + PACKAGEDROP_OFFSET and theta_diff < ANG_THRESHOLD:
    servo.min()
    sleep(5)
    servo.max()
    destination = ORIGIN
