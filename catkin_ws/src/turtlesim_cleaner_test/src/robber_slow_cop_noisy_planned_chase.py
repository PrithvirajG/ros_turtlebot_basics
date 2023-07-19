#!/usr/bin/env python3
import time
import message_filters
import math
import random
import numpy as np
from turtlesim.srv import *
from pid_controller import PID

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

import threading


class RobberTurtle:
    def __init__(self):
        # publishing topics
        self.publish_turtle = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)  # turtle commands
        self.pos_publish_real = rospy.Publisher("/rt_real_pose", Pose, queue_size=10)  # turtle real pose
        self.pos_publish_noise = rospy.Publisher("/rt_noisy_pose", Pose, queue_size=10)  # turtle noisy pose

        # define twist object for publishing actions
        self.msg = Twist()

        self.stop_msg = Twist()
        self.stop_msg.linear.x = 0
        self.stop_msg.angular.z = 0

    @staticmethod
    def limit_linear_acceleration(v_final, v_initial, t_initial):
        # define max limits of acceleration and deceleration
        max_acc_linear = 500
        max_dec_linear = -500

        # change in velocity
        dv = v_final - v_initial
        t_final = time.time()

        # change in time
        dt = t_final - t_initial

        # check if acceleration exceeds the limit
        if dv / dt > max_acc_linear:
            # calculate the new velocity
            dv2 = max_acc_linear * dt
            v_final = v_initial + dv2

        # check if deceleration exceeds the lower limit
        if dv / dt < max_dec_linear:
            # calculate the new velocity
            dv2 = max_dec_linear * dt
            v_final = v_initial + dv2

        return v_final, t_final

    def gaussian_noise(self):
        # generate gaussian noise on position of turtle

        # define the Pose object
        _pn = Pose()

        # standard deviation
        std = 0.3

        # with keeping mean as the actual value and standard deviation as 10 assign the below values to the Pose object
        _pn.x = np.random.normal(self.current_pose.x, std)
        _pn.y = np.random.normal(self.current_pose.y, std)
        _pn.theta = self.current_pose.theta + np.random.normal(self.current_pose.theta, std)
        _pn.angular_velocity = self.current_pose.x + np.random.normal(self.current_pose.angular_velocity, std)
        _pn.linear_velocity = self.current_pose.x + np.random.normal(self.current_pose.linear_velocity, std)

        # publish the noisy pose at '/rt_noisy_pose'
        self.pos_publish_noise.publish(_pn)

    def start_circle(self):
        # get the user input
        self.radius = 5
        self.speed = 2

        # define the tangential velocity of turtle bot
        self.msg.linear.x = self.speed

        # define the angular velocity of turtle bot
        self.msg.angular.z = self.msg.linear.x / self.radius

        # initialize variables
        initial_t = start_time = time.time()
        initial_vel = self.msg.linear.x

        while not rospy.is_shutdown():
            # initialize global variables of position
            global rt_pose_data, caught, pt_pose_data
            self.current_pose = rt_pose_data

            # define target speed
            final_vel = self.speed

            # limit acceleration and deceleration
            initial_vel, initial_t = self.limit_linear_acceleration(final_vel, initial_vel, initial_t)

            # assign linear speed value to the Twist object
            self.msg.linear.x = initial_vel
            if caught:
                self.publish_turtle.publish(self.stop_msg)
                break

            # publish the value to command the bot
            self.publish_turtle.publish(self.msg)

            # check if 5 seconds are passed
            if (time.time() - start_time) > 5 and self.current_pose is not None:
                start_time = time.time()

                # publish the position every 5 seconds
                self.pos_publish_real.publish(self.current_pose)

                # publish the gaussian noise
                self.gaussian_noise()


class PoliceTurtle:
    def __init__(self):
        # publish commands
        self.publish_turtle = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)  # turtle commands

        # initialize PID values
        self.angle_PID = PID(P=6, I=0, D=1)
        self.distance_PID = PID(P=0.7, I=0, D=0.1)

        # maximum distance between police and robber to consider it caught
        self.threshold_distance = 2

        self.triangle_abc = []

        # define twist object for publishing actions
        self.msg = Twist()
        self.distance = 100
        self.stop_msg = Twist()
        self.stop_msg.linear.x = 0
        self.stop_msg.angular.z = 0

    def calculate_robber_circle(self, A, B, C):
        D = 2*(A.x*(B.y - C.y) + B.x*(C.y - A.y) + C.x*(A.y - B.y))
        Ux = (A.x**2 + A.y**2)*(B.y - C.y) + (B.x**2 + B.y**2)*(C.y - A.y) + (C.x**2 + C.y**2)*(A.y - B.y)
        Uy = (A.x**2 + A.y**2)*(C.x - B.x) + (B.x**2 + B.y**2)*(A.x - C.x) + (C.x**2 + C.y**2)*(B.x - A.x)
        Ux = Ux/D
        Uy = Uy/D

        centre = (Ux, Uy)

        if Ux > 11 or Ux < 0 or Uy > 11 or Uy < 0:
            centre = (5.45, 5.45)

        radiusA = self.euclidian_distance(centre, (A.x, A.y))
        radiusB = self.euclidian_distance(centre, (B.x, B.y))
        radiusC = self.euclidian_distance(centre, (C.x, C.y))
        radius = (radiusA + radiusB + radiusC)/3
        print(f'Travelling on A CIRCLE : {centre} | {radius} |{radiusA}| {radiusB}| {radiusC}')

        return centre, radius

    def future_co_ordinates(self, centre, R, C):
        estimation_error = 0.2
        current_angle = math.atan2(C.y - centre[1], C.x - centre[0])
        future_angle = math.atan2(C.y - centre[1], C.x - centre[0]) + 2 + estimation_error
        future_co_ordinates = (centre[0] + R*math.cos(future_angle), centre[1] + R*math.sin(future_angle))
        distance = self.euclidian_distance(future_co_ordinates, centre)
        print(f'FUTURE_COORDINATES: {future_co_ordinates} | DISTANCE: {distance}')
        return future_co_ordinates

    @staticmethod
    def euclidian_distance(A, B):
        return math.sqrt((A[0] - B[0])**2 + (A[1] - B[1])**2)

    def start_chase(self):
        # spawn the police turtle
        self.spawn()

        # initial goal parameters
        self.goal_x = 5.45
        self.goal_y = 5.45

        start_time = time.time()
        _ = 100
        chasing = False
        centre = None
        while not rospy.is_shutdown():
            # initialize global variables
            global pt_pose_data, rt_pose_data, caught, rt_noise_pose

            if time.time() - start_time >= 5:
                current_target_real = rt_noise_pose
                self.msg = Twist()
                self.angle_PID = PID(P=6, I=0, D=1)
                self.distance_PID = PID(P=0.7, I=0, D=0.1)
                if rt_noise_pose is not None:
                    print(rt_noise_pose)
                    start_time = time.time()
                    self.triangle_abc.append(rt_noise_pose)
                    if len(self.triangle_abc) >= 4 and not chasing:
                        centre, radius = self.calculate_robber_circle(self.triangle_abc[-1], self.triangle_abc[-2], self.triangle_abc[-3])

            # start the movement when subscribed data is received
            if pt_pose_data is None or rt_noise_pose is None or centre is None:
                continue

            # if distance between cop and robber is less than threshold, stop
            if ((pt_pose_data.x - rt_pose_data.x) ** 2 + (pt_pose_data.y - rt_pose_data.y) ** 2) < self.threshold_distance ** 2:
                caught = True
                self.publish_turtle.publish(self.stop_msg)
                print('CAUGHT')
                break

            if not chasing:
                target_x, target_y = self.future_co_ordinates(centre, radius, current_target_real)
                _ = self.euclidian_distance((target_x, target_y), (pt_pose_data.x, pt_pose_data.y))
                print(f'CHASING {chasing}', _)
                if _ <= radius:
                    print(f'ROBBER WITHIN RANGE {_} : STARTING CHASE')
                    self.goal_x = target_x
                    self.goal_y = target_y
                    chasing = True

            if chasing:
                max_vel = 1

                # start movement of police turtle
                self.control_distance_new(max_vel)
                self.control_angle_new()
                self.publish_turtle.publish(self.msg)

                # halt the police turtle if reached at latest received robber co-ordinates
                if self.distance < 0.5 and len(self.triangle_abc) > 3:
                    chasing = False
                    self.publish_turtle.publish(self.stop_msg)
        print('FINISHED CHASE')

    def control_angle_new(self):
        target_angle = math.atan2(self.goal_y - pt_pose_data.y, self.goal_x - pt_pose_data.x)
        angle_difference = target_angle - pt_pose_data.theta
        self.PID_angle = self.angle_PID.update(angle_difference)
        self.msg.angular.z = self.PID_angle

    def control_distance_new(self, max_vel):
        self.distance = math.sqrt((self.goal_x - pt_pose_data.x) ** 2 + (self.goal_y - pt_pose_data.y) ** 2)
        self.PID_distance = self.distance_PID.update(self.distance)
        if self.PID_distance > max_vel:
            self.msg.linear.x = max_vel

    def spawn(self):
        x = random.random() * 10
        y = random.random() * 10
        theta = random.random() * math.pi
        print(f"Waiting for 10 seconds before spawning PT!")
        rospy.wait_for_service('/spawn')
        time.sleep(10)
        try:
            serv = rospy.ServiceProxy('/spawn', Spawn)
            serv(4, 5, theta, 'turtle2')
        except rospy.ServiceException as e:
            rospy.loginfo("Service execution failed: %s" + str(e))


def listener():
    # to subscribe all the topics for turtle position
    rospy.Subscriber("/turtle1/pose", Pose, robber_pose_callback)
    rospy.Subscriber("/turtle2/pose", Pose, police_pose_callback)
    rospy.Subscriber("/rt_noisy_pose", Pose, robber_noise_callback)
    rospy.spin()


def robber_noise_callback(data):
    # call back for position of robber turtles which is updated every 5 seconds
    global rt_noise_pose
    rt_noise_pose = data


def police_pose_callback(data):
    # call back for position of police turtle
    global pt_pose_data
    pt_pose_data = data


def robber_pose_callback(data):
    # call back for position of robber turtle
    global rt_pose_data
    rt_pose_data = data


if __name__ == '__main__':
    try:
        # global variables for position and caught status of turtles
        pt_pose_data = None
        rt_noise_pose = None
        rt_pose_data = None
        caught = False

        # initialize node
        rospy.init_node('robot_cleaner2', anonymous=True)
        rate = rospy.Rate(10)  # 10hz

        # define objects of turtles
        rt = RobberTurtle()
        pt = PoliceTurtle()

        # start the listener thread to subscribe the topics
        listener_thread = threading.Thread(target=listener)
        listener_thread.start()

        # start the robber thread to start movement of robber
        worker1 = threading.Thread(target=rt.start_circle)
        worker1.start()

        # start police thread to start the chase
        worker2 = threading.Thread(target=pt.start_chase)
        worker2.start()

        # join the threads
        listener_thread.join()
        worker1.join()
        worker2.join()

    except rospy.ROSInterruptException:
        pass
