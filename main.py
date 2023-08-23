# from CONFIG import *
from Classes import *
#from Ultrasonic import Ultrasonic
# from ShaftEncoder import ShaftEncoder
import numpy as np
from multiprocessing import Process, Value, Array
import gpiozero
from matplotlib import pyplot as plt
import time
import math
import argparse
WHEEL_SEPARATION = 220
WHEEL_RADIUS = 28
# NOTE MAKE ALL GLOBAL MULTITHREAD
last_time = time.time()
# Array [Left, Right]
last_encoder_steps = [0, 0]


def main(x_in, y_in, th_in):
    encoder_l = gpiozero.RotaryEncoder(a=17, b=27, max_steps=100000)
    encoder_r = gpiozero.RotaryEncoder(a=20, b=21, max_steps=100000)
    # 'i' for signed integer, 'd' for double prec float
    # ultra_arr = Array('i', [0, 0, 0])

    # TH_General = Process(target = )
    # TH_Ultrasonic = Process(target=Ultrasonic, args=(ultra_arr))
    # TH_Encoder = Process()

    # TH_General.start()
    # TH_Ultrasonic.start()
    robot = DiffDriveRobot(
        dt=0.1, wheel_radius=WHEEL_RADIUS, wheel_sep=WHEEL_SEPARATION)
    controller = RobotController(
        Kp=1.0, Ki=0.15, wheel_radius=WHEEL_RADIUS, wheel_sep=WHEEL_SEPARATION)
    planner = TentaclePlanner(dt=0.1, steps=5, alpha=1, beta=1e-5)
    motor_l = Motor(5, 6, 12)
    motor_r = Motor(23, 24, 13)

    poses = []
    velocities = []
    duty_cycle_commands = []

    goal_x = x_in  # NEEDS CHANGING
    goal_y = y_in  # NEEDS CHANGING
    goal_th = th_in  # NEEDS CHANGING

    for i in range(200):

        # Plan using tentacles
        v, w = planner.plan(goal_x, goal_y, goal_th,
                            robot.x, robot.y, robot.th)

        duty_cycle_l, duty_cycle_r = controller.drive(v, w, robot.wl, robot.wr)
        encoder_l_w, encoder_r_w = speedcalc(encoder_l, encoder_r)
        x, y, th = robot.pose_update(
            [encoder_l_w, encoder_r_w])  # NEED TO FIX
        motor_l.forward(duty_cycle_l)
        motor_r.forward(duty_cycle_r)
        # Log data
        poses.append([x, y, th])
        duty_cycle_commands.append([duty_cycle_l, duty_cycle_r])
        velocities.append([robot.wl, robot.wr])


def speedcalc(encoder_l, encoder_r):

   # Get the difference between last call and current call
    time_diff = time.time()-last_time
    encoder_l_diff = encoder_l.steps-last_encoder_steps[0]
    encoder_r_diff = encoder_r.steps-last_encoder_steps[1]

    # Update values
    last_time = time.time()
    last_encoder_steps[0] = encoder_l.steps
    last_encoder_steps[1] = encoder_r.steps
    encoder_l_w = 2*math.pi*(encoder_l_diff/920)/time_diff
    encoder_r_w = 2*math.pi*(encoder_r_diff/920)/time_diff
    return encoder_l_w, encoder_r_w


if __name__ == "__main__":
    parser = argparse.ArgumentParser("main")
    parser.add_argument("--x", type=int, default=10)
    parser.add_argument("--y", type=int, default=10)
    parser.add_argument("--th", type=float, default=0)
    args, _ = parser.parse_known_args
    main(args.x, args.y, args.th)
