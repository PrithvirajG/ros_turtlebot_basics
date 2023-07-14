#!/usr/bin/env python3
import time

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class ControlTurtleCirclesGoal3:
    def __init__(self):
        # publishing topics
        self.publish_turtle = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)  # turtle commands
        self.pos_publish_real = rospy.Publisher("/rt_real_pose", Pose, queue_size=10)  # turtle real pose
        self.pos_publish_noise = rospy.Publisher("/rt_noisy_pose", Pose, queue_size=10)  # turtle noisy pose

        # subscribing topics
        rospy.Subscriber("turtle1/pose", Pose, self.pose_callback)

        # initialize node
        rospy.init_node('robot_cleaner', anonymous=True)

        self.rate = rospy.Rate(10)  # 10hz

        # define twist object for publishing actions
        self.msg = Twist()

        # start the circular movement
        self.start_circle()

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
            print('----------------MAX ACCELERATION REACHED------------------------')

        # check if deceleration exceeds the lower limit
        if dv / dt < max_dec_linear:
            # calculate the new velocity
            dv2 = max_dec_linear * dt
            v_final = v_initial + dv2
            print('----------------MAX DECELERATION REACHED------------------------')

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
        self.get_user_input()

        # define the tangential velocity of turtle bot
        self.msg.linear.x = self.speed

        # define the angular velocity of turtle bot
        self.msg.angular.z = self.msg.linear.x / self.radius

        # initialize variables
        initial_t = start_time = time.time()
        initial_vel = self.msg.linear.x

        while not rospy.is_shutdown():
            # define target speed
            final_vel = self.speed

            # limit acceleration and deceleration
            initial_vel, initial_t = self.limit_linear_acceleration(final_vel, initial_vel, initial_t)

            # assign linear speed value to the Twist object
            self.msg.linear.x = initial_vel

            # publish the value to command the bot
            self.publish_turtle.publish(self.msg)

            # check if 5 seconds are passed
            if (time.time() - start_time) > 5:
                start_time = time.time()
                print('---->', self.current_pose_x, self.current_pose_y, self.current_angle)

                # publish the position every 5 seconds
                self.pos_publish_real.publish(self.current_pose)

                # publish the gaussian noise
                self.gaussian_noise()

    def get_user_input(self):
        # get user input for radius of circle and speed of turtle-bot
        self.radius = float(input("Radius of Circle: "))
        self.speed = float(input("Speed: "))

    def pose_callback(self, data):
        # call back function to save the subscribed data from 'turtle1/pose'
        self.current_pose_x = data.x
        self.current_pose_y = data.y
        self.current_angle = data.theta
        self.current_pose = data


if __name__ == '__main__':
    try:
        # Start the function
        ControlTurtleCirclesGoal3()
    except rospy.ROSInterruptException:
        pass
