#!/usr/bin/env python3
import math
import random
import threading
import time

import message_filters
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import *

from pid_controller import PID


class RobberTurtle:
    def __init__(self):
        # publishing topics
        self.publish_turtle = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)  # turtle commands
        self.pos_publish_real = rospy.Publisher("/rt_real_pose", Pose, queue_size=10)  # turtle real pose
        self.pos_publish_noise = rospy.Publisher("/rt_noisy_pose", Pose, queue_size=10)  # turtle noisy pose

        # subscribing topics
        rospy.Subscriber("turtle1/pose", Pose, self.pose_callback)

        # define twist object for publishing actions
        self.msg = Twist()

        self.stop_msg = Twist()
        self.stop_msg.linear.x = 0
        self.stop_msg.angular.z = 0

        # start the circular movement
        # self.start_circle()

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
        self.speed = 1

        # define the tangential velocity of turtle bot
        self.msg.linear.x = self.speed

        # define the angular velocity of turtle bot
        self.msg.angular.z = self.msg.linear.x / self.radius

        # initialize variables
        initial_t = start_time = time.time()
        initial_vel = self.msg.linear.x
        counter = 0

        while not rospy.is_shutdown():
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

            # print(f'-- {rt_pose_data}--')
            # check if 5 seconds are passed
            if (time.time() - start_time) > 5 and self.current_pose is not None:
                counter += 1
                start_time = time.time()

                # publish the position every 5 seconds
                self.pos_publish_real.publish(self.current_pose)

                # publish the gaussian noise
                self.gaussian_noise()
                if counter > 6:
                    break

    def pose_callback(self, data):
        # call back function to save the subscribed data from 'turtle1/pose'
        global rt_pose_data
        self.current_pose_x = data.x
        self.current_pose_y = data.y
        self.current_angle = data.theta
        rt_pose_data = self.current_pose = data


class PoliceTurtle:
    def __init__(self):
        # publish commands
        self.publish_turtle = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)  # turtle commands

        self.angle_PID = PID(P=6, I=0, D=1)
        self.distance_PID = PID(P=0.7, I=0, D=0.1)

        # define twist object for publishing actions
        self.msg = Twist()
        self.distance = 100
        self.stop_msg = Twist()
        self.stop_msg.linear.x = 0
        self.stop_msg.angular.z = 0

        # start the circular movement

    def start_chase(self):
        # spawn the cop turtle
        self.spawn()

        # start the chase
        start_time = time.time()
        while not rospy.is_shutdown():
            global pt_pose_data, rt_pose_data, caught
            if None in [pt_pose_data, rt_real_pose, rt_pose_data]:
                continue

            # fetch the new co-ordinates of robber turtle every 5 seconds
            if time.time() - start_time >= 5:
                self.msg = Twist()
                self.angle_PID = PID(P=5, I=0, D=0.1)
                self.distance_PID = PID(P=0.2, I=0, D=0.1)
                self.goal_x = rt_real_pose.x
                self.goal_y = rt_real_pose.y
                start_time = time.time()

            # control the movement of cop turtle
            self.control_distance_new()
            self.control_angle_new()
            self.publish_turtle.publish(self.msg)

            # halt when it reaches a point (the target where the robber turtle was present 5s ago)
            if self.distance < 0.5:
                self.publish_turtle.publish(self.stop_msg)

            # stop the loop once robber turtle is within range of cop turtle
            if ((pt_pose_data.x - rt_pose_data.x) ** 2 + (pt_pose_data.y - rt_pose_data.y) ** 2) < 1:
                caught = False
                self.publish_turtle.publish(self.stop_msg)
                break

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
        # spawns the cop turtle after 10 seconds
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
    # subscribe to other topics
    rt_real_pose_sub = message_filters.Subscriber("/rt_real_pose", Pose)
    turtle2_pos_sub = message_filters.Subscriber("/turtle2/pose", Pose)

    # filter the timestamp based on current time
    ts = message_filters.ApproximateTimeSynchronizer([turtle2_pos_sub, rt_real_pose_sub], 10, slop=0.1,
                                                     allow_headerless=True)
    ts.registerCallback(callback)

    rospy.spin()


def callback(data, data2):
    # store the callback in global variables
    global pt_pose_data, rt_real_pose
    pt_pose_data = data
    rt_real_pose = data2


if __name__ == '__main__':
    try:
        # Initialize global variables
        pt_pose_data = None
        rt_real_pose = None
        rt_pose_data = None
        caught = False

        # Initialize node
        rospy.init_node('robot_cleaner2', anonymous=True)
        rate = rospy.Rate(10)  # 10hz

        # Create the Turtle Objects
        rt = RobberTurtle()
        pt = PoliceTurtle()

        # start the listen thread to subscribe the topics
        listener_thread = threading.Thread(target=listener)
        listener_thread.start()

        # start the movement of robber turtle
        worker1 = threading.Thread(target=rt.start_circle)
        worker1.start()

        # start the chasing function of police startle
        worker2 = threading.Thread(target=pt.start_chase)
        worker2.start()

        # join the threads
        listener_thread.join()
        worker1.join()
        worker2.join()

    except rospy.ROSInterruptException:
        pass
