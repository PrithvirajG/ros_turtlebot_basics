#!/usr/bin/env python3
import math
import time

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class PID:
    def __init__(self, P=2.0, I=0.0, D=1.0, derivative=0, integral=0, integral_max=500, integral_min=-500):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.derivative = derivative
        self.integral = integral
        self.integral_max = integral_max
        self.integral_min = integral_min

        self.set_point = 0.0
        self.error = 0.0

    def update(self, error):
        self.error = error

        self.P_value = self.Kp * self.error
        self.D_value = self.Kd * (self.error - self.derivative)
        self.derivative = self.error

        self.integral = self.integral + self.error

        if self.integral > self.integral_max:
            self.integral = self.integral_max
        elif self.integral < self.integral_min:
            self.integral = self.integral_min

        self.I_value = self.integral * self.Ki

        PID = self.P_value + self.I_value + self.D_value

        return PID

    def setKp(self, P):
        self.Kp = P

    def setKi(self, I):
        self.Ki = I

    def setKd(self, D):
        self.Kd = D


class ControlTurtleGoal1:
    def __init__(self):
        self.publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("turtle1/pose", Pose, self.pose_callback)
        rospy.init_node('robot_cleaner', anonymous=True)

        self.rate = rospy.Rate(10)  # 10hz

        self.angle_PID = PID()
        self.distance_PID = PID()

        self.angle_PID.setKp(1.7)
        self.angle_PID.setKi(0.000002)
        self.angle_PID.setKd(10.9)

        self.distance_PID.setKp(0.9)
        self.distance_PID.setKi(0)
        self.distance_PID.setKd(0)

        self.msg = Twist()
        self.move_turtle()

    def control_angle(self):
        quadrants = {(True, True): (0, 1), (False, True): (math.pi, 1), (False, False): (-math.pi, 1), (True, False): (0, 1)}

        target_angle = math.atan((self.goal_y - self.current_pose_y)/(self.goal_x - self.current_pose_x))
        q = ((self.goal_x - self.current_pose_x) >= 0, (self.goal_y - self.current_pose_y) >= 0)
        print(q, quadrants[q])
        actual_target_angle = quadrants[q][0] + quadrants[q][1]*target_angle
        angle_difference = actual_target_angle - self.current_angle
        while abs(angle_difference) > 0.001:
            # time.sleep(3)
            target_angle = math.atan((self.goal_y - self.current_pose_y) / (self.goal_x - self.current_pose_x))
            q = ((self.goal_x - self.current_pose_x) >= 0, (self.goal_y - self.current_pose_y) >= 0)
            actual_target_angle = quadrants[q][0] + quadrants[q][1] * target_angle
            angle_difference = actual_target_angle - self.current_angle
            print('angle_difference: ', angle_difference, '\t', 'angle_difference: ', angle_difference)
            self.PID_angle = self.angle_PID.update(angle_difference)
            self.msg.angular.z = self.PID_angle
            self.publisher.publish(self.msg)
        time.sleep(2)
        print('current_angle', self.current_angle)
        print('current_angle degrees', math.degrees(self.current_angle))

    def control_distance(self):
        self.distance = math.sqrt((self.goal_x - self.current_pose_x)**2 + (self.goal_y - self.current_pose_y)**2)
        previous_distance = self.distance
        print("distance: " + str(self.distance))
        while self.distance > 0.05:
            self.distance = math.sqrt(
                math.pow(self.goal_x - self.current_pose_x, 2) + math.pow(self.goal_y - self.current_pose_y, 2))
            if previous_distance < self.distance and self.distance > 0.05:
                self.msg.linear.x = 0
                self.publisher.publish(self.msg)
                time.sleep(2)
                self.control_angle()
            previous_distance = self.distance
            print("distance: " + str(self.distance))
            self.PID_distance = self.distance_PID.update(self.distance)
            self.msg.linear.x = self.PID_distance
            self.publisher.publish(self.msg)
        time.sleep(2)

    def get_user_input(self):
        self.goal_x = float(input("X position:"))
        self.goal_y = float(input("Y position:"))
        # self.goal_y = self.current_pose_y

    def move_turtle(self):
        self.get_user_input()
        self.control_angle()
        self.control_distance()

    def pose_callback(self, data):
        self.current_pose_x = data.x
        self.current_pose_y = data.y
        self.current_angle = data.theta

if __name__ == '__main__':
    try:
        # Testing our function
        ControlTurtleGoal1()
    except rospy.ROSInterruptException:
        pass
