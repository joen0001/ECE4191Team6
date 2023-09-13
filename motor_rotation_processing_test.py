import time
import gpiozero
from Classes.Motor import Motor
import math
import multiprocessing

def angle_rot(encoder_steps, angular_velocity):
    while True:
        old_encoder_l = encoder_steps[0]
        old_encoder_r = encoder_steps[1]
        old_time = time.time()
        time.sleep(0.2)
        elapsed_time = time.time()-old_time
        angular_velocity[0] = 2 * math.pi * ((encoder_steps[0]-old_encoder_l)/960)/elapsed_time
        angular_velocity[1] = 2 * math.pi * ((encoder_steps[1]-old_encoder_r)/960)/elapsed_time

def main(motor_control, angular_velocity, encoder_steps):
    motor_control[0] = 1
    while True:
        print(f"w: {angular_velocity[0]}")
        print(f"steps: {encoder_steps[0]}")
        time.sleep(1)


if __name__ == "__main__":
    # Initialise the Motors and Encoders
    motor_l = Motor(5, 6, 12,17,27)
    motor_r = Motor(23, 24, 13, 20, 21)
    # Define a motor scaling factor to determine max speed as a fraction of PWM
    motor_speed_scaling = 0.3

    # Define all the shared variables
    encoder_steps = multiprocessing.Array('i', [0,0])
    motor_control = multiprocessing.Array('d', [0,0])
    angular_velocity = multiprocessing.Array('d', [0,0])

    # Defines the processes and begins them
    angle_rot_p = multiprocessing.Process(target=angle_rot, args=(encoder_steps, angular_velocity))
    main_p = multiprocessing.Process(target=main, args=(motor_control, angular_velocity, encoder_steps))
    angle_rot_p.start()
    main_p.start()

    while True:
        # Drive the motors at the current speed
        motor_l.drive(motor_control[0]*motor_speed_scaling)
        motor_r.drive(motor_control[1]*motor_speed_scaling)

        # Read and update the encoder steps
        encoder_steps[0] = motor_l.encoder.steps
        encoder_steps[1] = motor_r.encoder.steps

        # TODO: Read and save the ultrasonic