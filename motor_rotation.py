import time
import gpiozero
from Classes import Motor
import math
import multiprocessing

if __name__ == "__main__":
    # Initialise the Motors and Encoders
    motor_l = Motor(5, 6, 12,17,27)
    motor_r = Motor(23, 24, 13, 20, 21)
    # Define a motor scaling factor to determine max speed as a fraction of PWM
    motor_speed_scaling = 0.3

    motor_l.drive(0.3)

    while True:
        old_encoder_l = motor_l.encoder.steps
        old_encoder_r = motor_r.encoder.steps
        old_time = time.time()
        time.sleep(0.2)
        elapsed_time = time.time()-old_time
        print(motor_l.encoder.steps-old_encoder_l/960)
        print(elapsed_time)
        angular_velocity_l = ((motor_l.encoder.steps-old_encoder_l)/960)/elapsed_time
        angular_velocity_r = ((motor_r.encoder.steps-old_encoder_r)/960)/elapsed_time

        print(f'angular: {angular_velocity_l}')
        print(f'steps: {motor_l.encoder.steps}')