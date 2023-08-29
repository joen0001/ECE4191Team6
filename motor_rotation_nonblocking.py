import time
import gpiozero
from Classes import Motor
import math
import multiprocessing
from collections import deque

if __name__ == "__main__":
    # Initialise the Motors and Encoders
    motor_l = Motor(5, 6, 12,17,27)
    motor_r = Motor(23, 24, 13, 20, 21)
    # Define a motor scaling factor to determine max speed as a fraction of PWM
    motor_speed_scaling = 0.3

    motor_l.drive(0.3)
    motor_r.drive(0.3)

    times = deque([time.time(), time.time()])
    encoder_step = deque([[0,0],[0,0]])

    while True:
        if times[-1] - times[0] >= 0.2:
            times.popleft()
            encoder_step.popleft()

        times.append(time.time())
        encoder_step.append([motor_l.encoder.steps, motor_r.encoder.steps])

        angular_velocity_l = ((encoder_step[-1][0]-encoder_step[0][0])/960)/(times[-1]-times[0])
        angular_velocity_r = ((encoder_step[-1][1]-encoder_step[0][1])/960)/(times[-1]-times[0])

        print(f'angular_l: {angular_velocity_l}')
        print(f'angular_r: {angular_velocity_r}')


        time.sleep(0.01)