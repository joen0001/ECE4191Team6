from gpiozero import AngularServo
from time import sleep

servo = AngularServo(17,min_angle=0,max_angle=10, initial_angle=0)

while True:
#     servo.angle = 0
#     sleep(500)
#     servo.angle = 40
#     sleep(500)
    # for i in range(0, 90,1):
    #     servo.angle = i
    #     sleep(0.01)
    # sleep(2)
    # for i in range(90,0, -1):
    #     servo.angle = i
    #     sleep(0.01)
    # sleep(2)
    sleep(2)
    servo.min()
    servo.angle = 1
    sleep(2)
    servo.max()
    