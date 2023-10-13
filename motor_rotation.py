import time
import gpiozero
from CONFIG import *
from Classes.Motor import Motor
import math
import multiprocessing

if __name__ == "__main__":
    # Initialise the Motors and Encoders
    motor_l = Motor(MOTOR_L_FORWARD, MOTOR_L_BACKWARD, MOTOR_L_ENABLE, ROTENC_LEFT_A, ROTENC_LEFT_B)
    motor_r = Motor(MOTOR_R_FORWARD, MOTOR_R_BACKWARD, MOTOR_R_ENABLE, ROTENC_RIGHT_A, ROTENC_RIGHT_B)
    # Define a motor scaling factor to determine max speed as a fraction of PWM
    motor_speed_scaling = 0.3

    motor_l.drive(0.3)
    motor_r.drive(0.3)

    while True:
        old_encoder_l = motor_l.encoder.steps
        old_encoder_r = motor_r.encoder.steps
        old_time = time.time()
        time.sleep(0.2)
        elapsed_time = time.time()-old_time
        angular_velocity_l = 2 * math.pi * ((motor_l.encoder.steps-old_encoder_l)/960)/elapsed_time
        angular_velocity_r = 2 * math.pi * ((motor_r.encoder.steps-old_encoder_r)/960)/elapsed_time

        print(f'angular_l: {angular_velocity_l}')
        print(f'angular_r: {angular_velocity_r}')
        #print(angular_velocity_r-angular_velocity_l)
