# from CONFIG import *
from Classes import Motor, DiffDriveRobot, RobotController, TentaclePlanner
#from Ultrasonic import Ultrasonic
# from ShaftEncoder import ShaftEncoder
import numpy as np
from multiprocessing import Process, Value, Array
import gpiozero
from matplotlib import pyplot as plt
import time
import math
import argparse
WHEEL_SEPARATION = 0.22
WHEEL_RADIUS = 0.028
# NOTE MAKE ALL GLOBAL MULTITHREAD



def main(x_in, y_in, th_in):
    last_time = time.time()
    # Array [Left, Right]
    last_encoder_steps = [0, 0]

    # Initialise the Motors and Encoders
    motor_l = Motor(5, 6, 12,17,27)
    motor_r = Motor(23, 24, 13, 20, 21)
    # Define a motor scaling factor to determine max speed as a fraction of PWM
    motor_speed_scaling = 0.3

    # 'i' for signed integer, 'd' for double prec float
    # ultra_arr = Array('i', [0, 0, 0])

    # TH_General = Process(target = )
    # TH_Ultrasonic = Process(target=Ultrasonic, args=(ultra_arr))
    # TH_Encoder = Process()

    # TH_General.start()
    # TH_Ultrasonic.start()
    robot = DiffDriveRobot(
        dt=0.2, wheel_radius=WHEEL_RADIUS, wheel_sep=WHEEL_SEPARATION)
    controller = RobotController(
        Kp=1.0, Ki=0.15, wheel_radius=WHEEL_RADIUS, wheel_sep=WHEEL_SEPARATION)
    planner = TentaclePlanner(dt=0.2, steps=5, alpha=1, beta=1e-5)

    poses = []
    velocities = []
    duty_cycle_commands = []

    goal_x = x_in  # NEEDS CHANGING
    goal_y = y_in  # NEEDS CHANGING
    goal_th = th_in  # NEEDS CHANGING

    for i in range(10000):
        old_encoder_l = motor_l.encoder.steps
        old_encoder_r = motor_r.encoder.steps
        old_time = time.time()
        time.sleep(0.2)
        elapsed_time = time.time()-old_time
        angular_velocity_l = 2 * math.pi * ((motor_l.encoder.steps-old_encoder_l)/960)/elapsed_time
        angular_velocity_r = 2 * math.pi * ((motor_r.encoder.steps-old_encoder_r)/960)/elapsed_time
        robot.wl = angular_velocity_l
        robot.wr = angular_velocity_r

        # Plan using tentacles
        v, w = planner.plan(goal_x, goal_y, goal_th,
                            robot.x, robot.y, robot.th)

        duty_cycle_l, duty_cycle_r = controller.drive(v, w, robot.wl, robot.wr)
        x, y, th = robot.pose_update(
            [angular_velocity_l, angular_velocity_r])  # NEED TO FIX
        motor_l.drive(duty_cycle_l*0.3)
        motor_r.drive(duty_cycle_r*0.3)
        # Log data
        poses.append([x, y, th])
        duty_cycle_commands.append([duty_cycle_l, duty_cycle_r])
        velocities.append([robot.wl, robot.wr])

        print(f'pose: {poses} \n')
        print(f'duty_cycle: {duty_cycle_commands} \n')
        print(f'velocities: {velocities} \n')

        if abs(x-goal_x)<0.01 and abs(y-goal_y) < 0.01:
            break

    plt.plot(np.array(poses)[:,0],np.array(poses)[:,1])
    plt.plot(x,y,'k',marker='+')
    plt.quiver(x,y,0.1*np.cos(th),0.1*np.sin(th))
    plt.plot(goal_x,goal_y,'x',markersize=5)
    plt.quiver(goal_x,goal_y,0.1*np.cos(goal_th),0.1*np.sin(goal_th))
    plt.xlim(-1,1)
    plt.ylim(-1,1)
    plt.xlabel('x-position (m)')
    plt.ylabel('y-position (m)')
    plt.savefig('path_taken.png')


def speedcalc(encoder_l, encoder_r, last_time, last_encoder_steps):

   # Get the difference between last call and current call
    cur = time.time()
    cur_l = encoder_l.steps
    cur_r = encoder_r.steps
    time_diff = cur-last_time
    encoder_l_diff = cur_l-last_encoder_steps[0]
    encoder_r_diff = cur_r-last_encoder_steps[1]

    # Update values
    last_encoder_steps[0] = encoder_l.steps
    last_encoder_steps[1] = encoder_r.steps
    encoder_l_w = 2*math.pi*(encoder_l_diff/920)/time_diff
    encoder_r_w = 2*math.pi*(encoder_r_diff/920)/time_diff
    return encoder_l_w, encoder_r_w, cur


if __name__ == "__main__":
    # parser = argparse.ArgumentParser("main")
    # parser.add_argument("--x", type=int, default=10)
    # parser.add_argument("--y", type=int, default=10)
    # parser.add_argument("--th", type=float, default=0)
    # args, _ = parser.parse_known_args
    main(0.3, 0.3, 0)

