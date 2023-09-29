from CONFIG import *
from Classes.RobotController import RobotController
from Classes.TentaclePlanner import TentaclePlanner
from Classes.DiffDriveRobot import DiffDriveRobot
from Classes.Motor import Motor
from Classes.Ultrasonic import UltrasonicSensor
import numpy as np
import time
import math
from matplotlib import pyplot as plt

# Constants
WHEEL_SEPARATION = 0.22
WHEEL_RADIUS = 0.028
MOTOR_SPEED_SCALING = 0.2  # Scaling factor to determine max speed as a fraction of PWM

def main(goals):
    last_time = time.time()
    last_encoder_steps = [0, 0]  # Array [Left, Right]

    # Initialize Components
    motor_l, motor_r, us = initialize_motors_and_sensors()

    # Initialize robot, controller, and planner
    robot = DiffDriveRobot(dt=0.1, wheel_radius=WHEEL_RADIUS, wheel_sep=WHEEL_SEPARATION)
    controller = RobotController(Kp=1.0, Ki=0.15, wheel_radius=WHEEL_RADIUS, wheel_sep=WHEEL_SEPARATION)
    planner = TentaclePlanner(us, dt=0.2, steps=5, alpha=1, beta=1e-5)

    # Data logging
    poses = []
    velocities = []
    duty_cycle_commands = []

    for goal in goals:
        goal_x, goal_y, goal_th = goal
        while True:
            print_sensor_distances(us)
            elapsed_time, angular_velocity_l, angular_velocity_r = compute_velocities(motor_l, motor_r, last_time)
            robot.wl, robot.wr = angular_velocity_l, angular_velocity_r
            v, w = planner.plan(goal_x, goal_y, goal_th, robot.x, robot.y, robot.th)
            
            print_velocity(v, w, angular_velocity_l, angular_velocity_r)
            
            duty_cycle_l, duty_cycle_r = controller.drive(v, w, robot.wl, robot.wr)
            x, y, th = robot.pose_update([angular_velocity_l, angular_velocity_r])
            drive_motors(motor_l, motor_r, duty_cycle_l, duty_cycle_r, MOTOR_SPEED_SCALING)
            
            log_data(poses, velocities, duty_cycle_commands, x, y, th, robot.wl, robot.wr, duty_cycle_l, duty_cycle_r)
            print_current_state(poses, duty_cycle_commands)
            
            if goal_achieved(x, y, th, goal_x, goal_y, goal_th):
                adjust_orientation_and_break(motor_l, motor_r, th, goal_th, goal_x, poses, robot)

    plot_results(poses, goal_x, goal_y, goal_th, x, y, th) 
    
    
def initialize_motors_and_sensors():
    """Initializes motors, encoders, and ultrasonic sensors."""
    motor_l = Motor(5, 6, 12, 17, 27)
    motor_r = Motor(23, 24, 13, 20, 21)

    # Define ultrasonic sensor class
    us = UltrasonicSensor(ULT_FRONTL_ECHO, ULT_FRONTL_TRIG,
                          ULT_FRONTR_ECHO, ULT_FRONTR_TRIG,
                          ULT_RIGHT_ECHO, ULT_RIGHT_TRIG,
                          ULT_LEFT_ECHO, ULT_LEFT_TRIG,
                          THRESHOLD_F, THRESHOLD_LR)

    return motor_l, motor_r, us 


def print_sensor_distances(us):
    """Prints the distances detected by the ultrasonic sensors."""
    print(us.front1_distance(), us.front2_distance(), us.fleft_distance(), us.fright_distance())


def compute_velocities(motor_l, motor_r, last_time):
    """Computes and returns elapsed time and angular velocities."""
    old_encoder_l, old_encoder_r, old_time = motor_l.encoder.steps, motor_r.encoder.steps, time.time()
    time.sleep(0.1)
    elapsed_time = time.time()-old_time
    angular_velocity_l = 2*math.pi*((motor_l.encoder.steps-old_encoder_l)/960)/elapsed_time
    angular_velocity_r = 2*math.pi*((motor_r.encoder.steps-old_encoder_r)/960)/elapsed_time
    return elapsed_time, angular_velocity_l, angular_velocity_r


def print_velocity(v, w, angular_velocity_l, angular_velocity_r):
    """Prints velocities."""
    print(f'v: {v}, w: {w}\n')
    print("Angular velocity: " + angular_velocity_l + "\n" )
    print("Angular velocity: " + angular_velocity_r + "\n")


def drive_motors(motor_l, motor_r, duty_cycle_l, duty_cycle_r, motor_speed_scaling):
    """Drives motors."""
    motor_l.drive(duty_cycle_l * motor_speed_scaling)
    motor_r.drive(duty_cycle_r * motor_speed_scaling)


def log_data(poses, velocities, duty_cycle_commands, x, y, th, wl, wr, duty_cycle_l, duty_cycle_r):
    """Logs data."""
    poses.append([x, y, th])
    velocities.append([wl, wr])
    duty_cycle_commands.append([duty_cycle_l, duty_cycle_r])


def print_current_state(poses, duty_cycle_commands):
    """Prints the current state."""
    print(f'pose: {poses[-1]}')
    print(f'duty_cycle: {duty_cycle_commands[-1]}')


def goal_achieved(x, y, th, goal_x, goal_y, goal_th):
    """Determines if the goal has been achieved."""
    if abs(x-goal_x)<0.05 and abs(y-goal_y)<0.05:
        print('arrived')
        time.sleep(3)
        return True
    return False


def adjust_orientation_and_break(motor_l, motor_r, th, goal_th, goal_x, poses, robot):
    """Adjusts orientation of the robot and breaks out of the loop if needed."""
    while abs(th + goal_th) > 0.05 and goal_x != 0:
        old_encoder_l, old_encoder_r, old_time = motor_l.encoder.steps, motor_r.encoder.steps, time.time()
        time.sleep(0.1)
        elapsed_time = time.time() - old_time
        angular_velocity_l = 2 * math.pi * ((motor_l.encoder.steps - old_encoder_l) / 960) / elapsed_time
        angular_velocity_r = 2 * math.pi * ((motor_r.encoder.steps - old_encoder_r) / 960) / elapsed_time
        robot.wl, robot.wr = angular_velocity_l, angular_velocity_r
        x, y, th = robot.pose_update([angular_velocity_l, angular_velocity_r])
        motor_l.drive(-0.15)
        motor_r.drive(0.15)
        print(f'pose: {poses[-1]}')
        poses.append([x, y, th])
        print(f'diff {abs(th + goal_th)}')
    motor_l.stop()
    motor_r.stop()
    time.sleep(1)
    return True


def plot_results(poses, goal_x, goal_y, goal_th, x, y, th):
    """Plots results."""
    plt.plot(np.array(poses)[:,0],np.array(poses)[:,1])
    plt.plot(x,y,'k',marker='+')
    plt.quiver(x,y,0.1*np.cos(th),0.1*np.sin(th))
    plt.plot(goal_x,goal_y,'x',markersize=5)
    plt.quiver(goal_x,goal_y,0.1*np.cos(goal_th),0.1*np.sin(goal_th))
    plt.xlim(-1,1)
    plt.ylim(-1,1)
    plt.show()
    

if __name__ == "__main__":
    goals = [(0.6,-0.6,math.pi),(0,-0.6,math.pi)]
    main(goals)

