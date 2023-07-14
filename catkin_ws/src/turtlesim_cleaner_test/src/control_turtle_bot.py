#!/usr/bin/env python3
import math
import time

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from pid_controller import PID


class ControlTurtleGoal1:
    def __init__(self):
        # publishing nodes
        self.publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # subscribing nodes
        rospy.Subscriber("turtle1/pose", Pose, self.pose_callback)

        # initialize node
        rospy.init_node('robot_cleaner', anonymous=True)

        self.rate = rospy.Rate(10)  # 10hz

        # define PID instance
        self.angle_PID = PID()
        self.distance_PID = PID()

        # set values of Kp, Ki, Kd for distance control
        self.angle_PID.setKp(1.7)
        self.angle_PID.setKi(0.000002)
        self.angle_PID.setKd(10.9)

        # set values of Kp, Ki, Kd for angle control
        self.distance_PID.setKp(0.9)
        self.distance_PID.setKi(0)
        self.distance_PID.setKd(0)

        # define twist object for publishing actions
        self.msg = Twist()

        # start the grid
        self.move_turtle()

    def control_angle(self):
        # quadrant of the angle relative to starting point
        quadrants = {(True, True): (0, 1), (False, True): (math.pi, 1), (False, False): (-math.pi, 1),
                     (True, False): (0, 1)}

        # target angle: angle from X-axis and reference as starting point
        target_angle = math.atan((self.goal_y - self.current_pose_y) / (self.goal_x - self.current_pose_x))

        # actual target angle: angle of orientation of bot + target angle
        _q = ((self.goal_x - self.current_pose_x) >= 0, (self.goal_y - self.current_pose_y) >= 0)
        print(_q, quadrants[_q])
        actual_target_angle = quadrants[_q][0] + quadrants[_q][1] * target_angle

        # angle difference
        angle_difference = actual_target_angle - self.current_angle

        # start the while loop with angle precision of 0.001 radians
        while abs(angle_difference) > 0.001:
            # calculate actual target angle
            target_angle = math.atan((self.goal_y - self.current_pose_y) / (self.goal_x - self.current_pose_x))
            _q = ((self.goal_x - self.current_pose_x) >= 0, (self.goal_y - self.current_pose_y) >= 0)
            actual_target_angle = quadrants[_q][0] + quadrants[_q][1] * target_angle

            # calculate angle difference
            angle_difference = actual_target_angle - self.current_angle
            print('angle_difference: ', angle_difference, '\t', 'angle_difference: ', angle_difference)

            # get the output from PID control
            self.PID_angle = self.angle_PID.update(angle_difference)

            # save the PID output to the angular velocity of the Twist object
            self.msg.angular.z = self.PID_angle

            # publish at the new Twist() values on turtle1 node
            self.publisher.publish(self.msg)

        # turtle bot needs to be halted completely to start the next action
        time.sleep(2)
        print('current_angle', self.current_angle)
        print('current_angle degrees', math.degrees(self.current_angle))

    def control_distance(self):
        # euclidian distance between current position and target position
        self.distance = math.sqrt((self.goal_x - self.current_pose_x) ** 2 + (self.goal_y - self.current_pose_y) ** 2)
        previous_distance = self.distance
        print("distance: " + str(self.distance))

        while self.distance > 0.05:
            # calculate distance
            self.distance = math.sqrt(
                math.pow(self.goal_x - self.current_pose_x, 2) + math.pow(self.goal_y - self.current_pose_y, 2))

            # if the robot misses the target by minute error in angle, we re-adjust here
            if previous_distance < self.distance and self.distance > 0.05:
                self.msg.linear.x = 0
                self.publisher.publish(self.msg)
                time.sleep(2)
                self.control_angle()

            # save the distance
            previous_distance = self.distance
            print("distance: " + str(self.distance))

            # get the output from PID control
            self.PID_distance = self.distance_PID.update(self.distance)

            # save the output to the linear velocity of the Twist object
            self.msg.linear.x = self.PID_distance

            # publish at the new Twist() values on turtle1 node
            self.publisher.publish(self.msg)
        time.sleep(2)

    def get_user_input(self):
        # get input values from user
        self.goal_x = float(input("X position:"))
        self.goal_y = float(input("Y position:"))

    def move_turtle(self):
        # start the movement of turtle
        self.get_user_input()
        self.control_angle()
        self.control_distance()

    def pose_callback(self, data):
        # call back function to save the subscribed data from 'turtle1/pose'
        self.current_pose_x = data.x
        self.current_pose_y = data.y
        self.current_angle = data.theta


if __name__ == '__main__':
    try:
        # Start the function
        ControlTurtleGoal1()
    except rospy.ROSInterruptException:
        pass
