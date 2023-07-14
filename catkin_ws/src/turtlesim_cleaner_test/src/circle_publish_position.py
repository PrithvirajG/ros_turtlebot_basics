#!/usr/bin/env python3
import math
import time
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class ControlTurtleCirclesGoal3:
    def __init__(self):
        self.publish_turtle = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pos_publish_real = rospy.Publisher("/rt_real_pose", Pose, queue_size=10)
        self.pos_publish_noise = rospy.Publisher("/rt_noisy_pose", Pose, queue_size=10)

        rospy.Subscriber("turtle1/pose", Pose, self.pose_callback)
        rospy.init_node('robot_cleaner', anonymous=True)

        self.rate = rospy.Rate(10)  # 10hz
        self.msg = Twist()

        self.start_circle()

    @staticmethod
    def limit_linear_acceleration(v_final, v_initial, t_initial):
        max_acc_linear = 500
        max_dec_linear = -500
        dv = v_final - v_initial
        t_final = time.time()
        dt = t_final - t_initial

        if dv/dt > max_acc_linear:
            dv2 = max_acc_linear * dt
            v_final = v_initial + dv2

        if dv/dt < max_dec_linear:
            dv2 = max_dec_linear * dt
            v_final = v_initial + dv2

        return v_final, t_final

    def gaussian_noise(self):
        _pn = Pose()
        _pn.x = np.random.normal(self.current_pose.x, 10)
        _pn.y = np.random.normal(self.current_pose.y, 10)
        _pn.theta = self.current_pose.theta + np.random.normal(self.current_pose.theta, 10)
        _pn.angular_velocity = self.current_pose.x + np.random.normal(self.current_pose.angular_velocity, 10)
        _pn.linear_velocity = self.current_pose.x + np.random.normal(self.current_pose.linear_velocity, 10)
        self.pos_publish_noise.publish(_pn)

    def start_circle(self):
        self.get_user_input()
        self.msg.linear.x = self.speed
        self.msg.angular.z = self.msg.linear.x/self.radius

        initial_t = start_time = time.time()
        initial_vel = self.msg.linear.x
        count = 0

        while not rospy.is_shutdown():
            final_vel = self.speed
            initial_vel, initial_t = self.limit_linear_acceleration(final_vel, initial_vel, initial_t)
            self.msg.linear.x = initial_vel
            self.publish_turtle.publish(self.msg)
            if (time.time() - start_time) > 5:
                count += 1
                start_time = time.time()
                print('---->', self.current_pose_x, self.current_pose_y, self.current_angle)
                self.pos_publish_real.publish(self.current_pose)
                self.gaussian_noise()

    def get_user_input(self):
        self.radius = float(input("Radius of Circle: "))
        self.speed = float(input("Speed: "))
        # self.goal_y = self.current_pose_y

    def pose_callback(self, data):
        self.current_pose_x = data.x
        self.current_pose_y = data.y
        self.current_angle = data.theta
        self.current_pose = data


if __name__ == '__main__':
    try:
        # Testing our function
        ControlTurtleCirclesGoal3()
    except rospy.ROSInterruptException:
        pass
