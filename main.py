from CONFIG import *
from Classes.RobotController import RobotController
from Classes.WallFollower import WallFollower
from Classes.TentaclePlanner import TentaclePlanner
from Classes.DiffDriveRobot import DiffDriveRobot
from Classes.Motor import Motor
from Classes.Ultrasonic import UltrasonicSensor
from Classes.RFID import RFIDScanner
import numpy as np
import time
import math
from matplotlib import pyplot as plt
import multiprocessing
import socket
from gpiozero import AngularServo
# Constants
WHEEL_SEPARATION = 0.22
WHEEL_RADIUS = 0.028
MOTOR_SPEED_SCALING = 0.2  # Scaling factor to determine max speed as a fraction of PWM

arr = multiprocessing.Array('d', [0.0,0.0,0.0])

def do_bluetooth(arr):
    s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    host_address = 'D8:3A:DD:21:87:12' #Fill in host mac address here
    s.bind((host_address,1))

    s.listen(1)
    try:
        client, address = s.accept()
        while 1:
            data = client.recv(1024)
            if data:
                print ('Their arr:', np.frombuffer(data))
            client.send(np.array(arr).tobytes())
        
            time.sleep(1)
    except:
        print("Closing socket")
        client.close()
        s.close()

def wall_run(goal):

    # p = multiprocessing.Process(target=do_bluetooth, args=(arr,))
    # p.start()

    last_time = time.time()
    # Initialize Components
    motor_l, motor_r, us,servo,rfid = initialize_motors_and_sensors()
    # Establishing new classes:
    robot = DiffDriveRobot(dt=0.1, wheel_radius=WHEEL_RADIUS, wheel_sep=WHEEL_SEPARATION)
    controller = RobotController(Kp=1.0, Ki=0.15, wheel_radius=WHEEL_RADIUS, wheel_sep=WHEEL_SEPARATION)
    waller = WallFollower(us)
    
    # Data logging
    poses = []
    velocities = []
    duty_cycle_commands = []
    previousAngle = 0
    integA = 0
    corner_counter = 0  # Counter to keep track of the number of corners
    th = 0
    x=0
    y=0
    y_wall = 0
    print('waiting for Beep')
    goal = rfid.BEEP()
    print('beep received')
    while True:
        elapsed_time, angular_velocity_l, angular_velocity_r = compute_velocities(motor_l, motor_r, last_time)
        robot.wl, robot.wr = angular_velocity_l, angular_velocity_r
        # Goal B:
        # TODO: Calculate required distance or use coordinates to ascertain goal B.
        
            
        # Loading bay: 
        # TODO: Calculate required distance or use coordinates to ascertain loading bay
        #if corner_counter == 4 and us.front1_distance() < 50:
        #    loading_bay()
        #    break
    
        # function will check ultrasonic sensors, corner
        # will return None if not at corner, will return 
        v, w = waller.is_at_corner()
        if v is not None:
            motor_l.stop()
            motor_r.stop()
            corner_counter += 1
            
            start_th = th
            if corner_counter%4==2 and goal=='C':
                motor_l.stop()
                motor_r.stop()
                time.sleep(1)
                servo.max()
                time.sleep(1)
                servo.min()
                print('A')
                goal = ""
            print('TURNING into corner'+str(corner_counter%4))
            print('TURNING into corner'+str(corner_counter%4))
            print('TURNING into corner'+str(corner_counter%4))
            print('TURNING into corner'+str(corner_counter%4))
            print('TURNING into corner'+str(corner_counter%4))
            while abs(start_th - th) < math.pi/2.3:
                old_encoder_l, old_encoder_r, old_time = motor_l.encoder.steps, motor_r.encoder.steps, time.time()
                time.sleep(0.1)
                elapsed_time = time.time() - old_time
                angular_velocity_l = 2 * math.pi * ((motor_l.encoder.steps - old_encoder_l) / 960) / elapsed_time
                angular_velocity_r = 2 * math.pi * ((motor_r.encoder.steps - old_encoder_r) / 960) / elapsed_time
                robot.wl, robot.wr = angular_velocity_l, angular_velocity_r
                x, y, th = robot.pose_update([angular_velocity_l, angular_velocity_r])
                motor_l.drive(0.15)
                motor_r.drive(-0.15)
                #print(f'pose: {poses[-1]}')
                poses.append([x, y, th])
                y_wall = y
            if corner_counter%4==0:
                motor_l.stop()
                motor_r.stop()
                goal = rfid.BEEP()
            if corner_counter%4==1 and goal=='A':
                motor_l.stop()
                motor_r.stop()
                time.sleep(1)
                servo.max()
                time.sleep(1)
                servo.min()
                print('A')
                goal = ""
            
        v, w,previousAngle,integA = waller.maintain_left_distance(previousAngle,integA)
        execute_drive_cycle(controller, robot, motor_l, motor_r, v, w, poses, velocities, duty_cycle_commands, MOTOR_SPEED_SCALING)

        #print('corner_counter:'+str(corner_counter))
        print('y' +str(poses[-1][1])+'     y_wall'+str(y_wall)+'    diff:'+str(poses[-1][1]-y_wall))
        if abs(poses[-1][1]-y_wall)>0.15 and corner_counter%4==1 and goal == 'B':
            #and front_sensor <0.5
            motor_l.stop()
            motor_r.stop()
            time.sleep(1)
            servo.max()
            time.sleep(1)
            servo.min()
            print('B')
            goal = ""
            #drop_package()
        time.sleep(0.1)

def execute_drive_cycle(controller, robot, motor_l, motor_r, v, w, poses, velocities, duty_cycle_commands, MOTOR_SPEED_SCALING):
    duty_cycle_l, duty_cycle_r = controller.drive(v, w, robot.wl, robot.wr)
    x, y, th = robot.pose_update([robot.wl, robot.wr])
    arr[0] = x
    arr[1] = y
    arr[2] = th
    print('x:'+str(x)+'y:'+str(y)+'theta:'+str(th))
    log_data(poses, velocities, duty_cycle_commands, x, y, th, robot.wl, robot.wr, duty_cycle_l, duty_cycle_r)
    drive_motors(motor_l, motor_r, duty_cycle_l+0.025, duty_cycle_r, MOTOR_SPEED_SCALING)
    return x,y,th  

def drop_package():
    print("This is where I would drop the package!")
    
def loading_bay():
    print("This is where I wait for my load ;)")

def tentacle_run(goals):
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
    motor_l = Motor(MOTOR_L_FORWARD, MOTOR_L_BACKWARD, MOTOR_L_ENABLE, ROTENC_LEFT_A, ROTENC_LEFT_B)
    motor_r = Motor(MOTOR_R_FORWARD, MOTOR_R_BACKWARD, MOTOR_R_ENABLE, ROTENC_RIGHT_A, ROTENC_RIGHT_B)
    servo = AngularServo(SERVO,min_angle=0,max_angle=10, initial_angle=0)
    rfid = RFIDScanner()
    # Define ultrasonic sensor class
    us = UltrasonicSensor(ULT_FRONT_ECHO,ULT_FRONT_TRIG,
                          ULT_SIDEFRONT_ECHO,ULT_SIDEFRONT_TRIG,
                          ULT_SIDEBACK_ECHO, ULT_SIDEBACK_TRIG,
                          )

    return motor_l, motor_r, us, servo,rfid


def print_sensor_distances(us):
    """Prints the distances detected by the ultrasonic sensors."""
    #print(us.front1_distance(), us.front2_distance(), us.fleft_distance())


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
    # tentacle_run(goals)
    wall_run("C")  # for Goal A
    # wall_run("B")  # for Goal B
    # wall_run("C")  # for Goal C
    # tentacle_run(0.3,0.3,0)


