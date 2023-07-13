#!/usr/bin/env python3
import math
import time

from std_msgs.msg import Float32

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


class ControlTurtleGridGoal2:
    def __init__(self):
        self.turtle_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.acc_pub = rospy.Publisher("/linear_acceleration", Float32, queue_size=10)
        rospy.Subscriber("turtle1/pose", Pose, self.pose_callback)
        rospy.init_node('robot_cleaner', anonymous=True)

        self.rate = rospy.Rate(10)  # 10hz

        self.angle_PID = PID()
        self.distance_PID = PID()

        self.angle_PID.setKp(1.7)
        self.angle_PID.setKi(0)
        self.angle_PID.setKd(10.9)

        self.distance_PID.setKp(0.9)
        self.distance_PID.setKi(0)
        self.distance_PID.setKd(1)

        self.msg = Twist()
        # self.start_grid()
        self.move_turtle()

    def control_angle(self):
        quadrants = {(True, True): (0, 1), (False, True): (math.pi, 1), (False, False): (-math.pi, 1), (True, False): (0, 1)}

        target_angle = math.atan((self.goal_y - self.current_pose_y)/(self.goal_x - self.current_pose_x))
        q = ((self.goal_x - self.current_pose_x) >= 0, (self.goal_y - self.current_pose_y) >= 0)
        # print(q, quadrants[q])
        actual_target_angle = quadrants[q][0] + quadrants[q][1]*target_angle
        angle_difference = actual_target_angle - self.current_angle
        while abs(angle_difference) > 0.001:
            q = ((self.goal_x - self.current_pose_x) >= 0, (self.goal_y - self.current_pose_y) >= 0)
            actual_target_angle = quadrants[q][0] + quadrants[q][1] * target_angle
            angle_difference = actual_target_angle - self.current_angle
            print('angle_difference: ', angle_difference, '\t', 'angle_difference: ', angle_difference)
            self.PID_angle = self.angle_PID.update(angle_difference)
            self.msg.angular.z = self.PID_angle
            self.turtle_pub.publish(self.msg)
        time.sleep(2)
        print('current_angle', self.current_angle)
        print('current_angle degrees', math.degrees(self.current_angle))

    def control_distance(self):
        self.distance = math.sqrt((self.goal_x - self.current_pose_x)**2 + (self.goal_y - self.current_pose_y)**2)
        previous_distance = self.distance

        print("distance: " + str(self.distance))
        initial_vel = self.msg.linear.x
        initial_t = time.time()

        while self.distance > 0.05:
            self.distance = math.sqrt(
                math.pow(self.goal_x - self.current_pose_x, 2) + math.pow(self.goal_y - self.current_pose_y, 2))
            if previous_distance < self.distance and self.distance > 0.05:
                self.msg.linear.x = 0
                self.turtle_pub.publish(self.msg)
                time.sleep(2)
                self.control_angle()

            previous_distance = self.distance
            print("distance: " + str(self.distance))

            self.PID_distance = self.distance_PID.update(self.distance)
            final_vel = self.PID_distance
            initial_vel, initial_t = self.limit_linear_acceleration(final_vel, initial_vel, initial_t)

            self.msg.linear.x = initial_vel
            self.turtle_pub.publish(self.msg)
        time.sleep(2)

    def limit_linear_acceleration(self, v_final, v_initial, t_initial):
        max_acc_linear = 500
        max_dec_linear = -500
        dv = v_final - v_initial
        t_final = time.time()
        dt = t_final - t_initial

        if dv/dt > max_acc_linear:
            dv2 = max_acc_linear * dt
            print('----------------MAX ACCELERATION REACHED------------------------')
            v_final = v_initial + dv2
            # print(f'v final new: {v_final} \n------------------------')

        if dv/dt < max_dec_linear:
            dv2 = max_dec_linear * dt
            print('----------------MAX DECELERATION REACHED------------------------')
            v_final = v_initial + dv2

        return v_final, t_final

    def start_grid(self):
        grid = [(1, 1), (10, 1), (10, 2), (1, 2), (1, 3), (10, 3), (10, 4), (1, 4), (1, 5),
                (10, 5), (10, 6), (1, 6), (1, 7), (10, 7), (10, 8), (1, 8), (1, 9), (10, 9), (10, 10)]

        for objective in grid:
            self.goal_x, self.goal_y = objective
            print(f'CURRENT OBJECTIVE: {self.goal_x}, {self.goal_y}')

            self.angle_PID = PID(P=1.7, I=0.000002, D=10.9)
            self.distance_PID = PID(P=0.9, I=0, D=0)
            self.msg = Twist()

            self.control_angle()
            self.control_distance()

    def pose_callback(self, data):
        self.current_pose_x = data.x
        self.current_pose_y = data.y
        self.current_angle = data.theta


if __name__ == '__main__':
    try:
        # Testing our function
        ControlTurtleGridGoal2()
    except rospy.ROSInterruptException:
        pass
