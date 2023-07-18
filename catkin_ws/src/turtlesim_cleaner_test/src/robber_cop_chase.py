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

        # with keeping mean as the actual value and standard deviation as 10 assign the below values to the Pose object
        _pn.x = np.random.normal(self.current_pose.x, 10)
        _pn.y = np.random.normal(self.current_pose.y, 10)
        _pn.theta = self.current_pose.theta + np.random.normal(self.current_pose.theta, 10)
        _pn.angular_velocity = self.current_pose.x + np.random.normal(self.current_pose.angular_velocity, 10)
        _pn.linear_velocity = self.current_pose.x + np.random.normal(self.current_pose.linear_velocity, 10)

        # publish the noisy pose at '/rt_noisy_pose'
        self.pos_publish_noise.publish(_pn)

    def start_circle(self):
        # get the user input
        self.radius = 4
        self.speed = 0.4

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

        # define twist object for publishing actions
        self.msg = Twist()
        self.distance = 100
        self.stop_msg = Twist()
        self.stop_msg.linear.x = 0
        self.stop_msg.angular.z = 0

        # start the circular movement
    def start_chase(self):
        # spawn the police turtle
        self.spawn()

        # initial goal parameters
        self.goal_x = 5.45
        self.goal_y = 5.45

        start_time = time.time()
        _ = 100
        while not rospy.is_shutdown():
            # initialize global variables
            global pt_pose_data, rt_pose_data, caught, rt_real_pose

            if time.time() - start_time >= 5:

                self.msg = Twist()
                self.angle_PID = PID(P=6, I=0, D=1)
                self.distance_PID = PID(P=0.7, I=0, D=0.1)
                if rt_real_pose is not None:
                    self.goal_x = rt_real_pose.x
                    self.goal_y = rt_real_pose.y
                    start_time = time.time()

                print(f'LATEST RECEIVED CO_ORDINATES OF ROBBER: ({self.goal_x}, {self.goal_y})')

            # start the movement when subscribed data is received
            if pt_pose_data is None:
                continue

            # start movement of police turtle
            self.control_distance_new()
            self.control_angle_new()
            self.publish_turtle.publish(self.msg)

            # halt the police turtle if reached at latest received robber co-ordinates
            if self.distance < 0.5:
                self.publish_turtle.publish(self.stop_msg)

            # if distance between cop and robber is less than threshold, stop
            if ((pt_pose_data.x - rt_pose_data.x)**2 + (pt_pose_data.y - rt_pose_data.y)**2) < self.threshold_distance**2:
                caught = True
                self.publish_turtle.publish(self.stop_msg)
                print('CAUGHT')
                break

        print('FINISHED CHASE')

    def control_angle_new(self):
        target_angle = math.atan2(self.goal_y - pt_pose_data.y, self.goal_x - pt_pose_data.x)
        angle_difference = target_angle - pt_pose_data.theta
        self.PID_angle = self.angle_PID.update(angle_difference)
        self.msg.angular.z = self.PID_angle

    def control_distance_new(self):
        self.distance = math.sqrt((self.goal_x - pt_pose_data.x) ** 2 + (self.goal_y - pt_pose_data.y) ** 2)
        self.PID_distance = self.distance_PID.update(self.distance)
        self.msg.linear.x = self.PID_distance

    def spawn(self):
        x = random.random() * 10
        y = random.random() * 10
        theta = random.random() * math.pi
        print(f"Waiting for 10 seconds before spawning PT!")
        rospy.wait_for_service('/spawn')
        time.sleep(10)
        try:
            serv = rospy.ServiceProxy('/spawn', Spawn)
            serv(x, y, theta, 'turtle2')
        except rospy.ServiceException as e:
            rospy.loginfo("Service execution failed: %s" + str(e))


def listener():
    # to subscribe all the topics for turtle position
    rospy.Subscriber("/turtle1/pose", Pose, robber_pose_callback)
    rospy.Subscriber("/turtle2/pose", Pose, police_pose_callback)
    rospy.Subscriber("/rt_real_pose", Pose, robber_real_callback)
    rospy.spin()


def robber_real_callback(data):
    # call back for position of robber turtles which is updated every 5 seconds
    global rt_real_pose
    rt_real_pose = data


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
        rt_real_pose = None
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
