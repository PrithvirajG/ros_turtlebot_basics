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

ROBBER_TURTLE_SPEED = 3
POLICE_TURTLE_SPEED = ROBBER_TURTLE_SPEED/2
RADIUS = 30


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
    def euclidian_distance(A, B):
        return math.sqrt((A[0] - B[0])**2 + (A[1] - B[1])**2)

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
        self.radius = RADIUS
        self.speed = ROBBER_TURTLE_SPEED

        # define the tangential velocity of turtle bot
        self.msg.linear.x = self.speed

        # define the angular velocity of turtle bot
        self.msg.angular.z = self.msg.linear.x / self.radius

        # initialize variables
        initial_t = start_time = time.time()
        initial_vel = self.msg.linear.x
        i = 0
        while not rospy.is_shutdown():
            i += 1
            # initialize global variables of position
            global robber_pose_data, caught, police_pose_data
            self.current_pose = robber_pose_data

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

            if self.current_pose is not None:
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
        self.threshold_distance = 3

        self.previous_co_ordinates_of_robber = []

        # define twist object for publishing actions
        self.msg = Twist()
        self.distance = 100
        self.stop_msg = Twist()
        self.stop_msg.linear.x = 0
        self.stop_msg.angular.z = 0

    @staticmethod
    def calculate_robber_circle_centre(robber_pose):
        cx = None
        cy = None
        if robber_pose.angular_velocity != 0:
            radius = robber_pose.linear_velocity / robber_pose.angular_velocity
            cx = robber_pose.x - radius * math.cos(robber_pose.theta - math.pi/2)
            cy = robber_pose.y - radius * math.sin(robber_pose.theta - math.pi/2)
        return (cx, cy)

    @staticmethod
    def euclidian_distance(A, B):
        return math.sqrt((A[0] - B[0])**2 + (A[1] - B[1])**2)

    @staticmethod
    def get_new_co_ordinates_of_robber(start_time):
        global robber_pose_data
        if time.time() - start_time >= 5:
            print(f'NEW CO_ORDINATES RECEIVED\n{robber_pose_data}\n')
            return robber_pose_data
        else:
            return False

    @staticmethod
    def check_if_turtle_stuck_at_edge(previous_co_ordinates_of_police, current_co_ordinates_of_police):
        if previous_co_ordinates_of_police is None or current_co_ordinates_of_police is None:
            return False
        if (abs(previous_co_ordinates_of_police.x - current_co_ordinates_of_police.x) < 0.01) or \
                (abs(previous_co_ordinates_of_police.y - current_co_ordinates_of_police.y) < 0.01):
            return True
        else:
            return False

    def start_chase(self):
        # spawn the police turtle
        self.spawn()

        # initial goal parameters
        self.goal_x = 5.45
        self.goal_y = 5.45

        # start time defined
        start_time = time.time()

        centre = None
        received_robber_pose = None
        chasing_key = False

        expected_time_to_reach = 0
        time_elapsed = 0
        previous_police_turtle_pose = None

        while not rospy.is_shutdown():

            # initialize global variables
            global police_pose_data, robber_pose_data, caught

            # fetch the new co-ordinates if available
            _ = self.get_new_co_ordinates_of_robber(start_time)

            # if fetched start with controlling
            if _:
                # save the robber turtle pose
                received_robber_pose = _
                start_time = time.time()

                # append to the list of previous co-ordinates
                self.previous_co_ordinates_of_robber.append(received_robber_pose)
                # self.previous_co_ordinates_of_police.append(police_pose_data)
                _ = False

                time_elapsed += 5

                # checking if cop has reached the edge
                if self.check_if_turtle_stuck_at_edge(police_pose_data, previous_police_turtle_pose):
                    chasing_key = False
                    print('POLICE STUCK ON EDGE')
                previous_police_turtle_pose = police_pose_data

            # start the movement when subscribed data is received
            if police_pose_data is None or received_robber_pose is None:
                continue

            # if distance between cop and robber is less than threshold, COMPLETED CHASE
            if self.euclidian_distance(
                    (police_pose_data.x, police_pose_data.y),
                    (robber_pose_data.x, robber_pose_data.y)) < self.threshold_distance:
                caught = True
                self.publish_turtle.publish(self.stop_msg)
                print('CAUGHT')
                break

            # if chasing has not started
            if not chasing_key:

                # initialize control variables of PID
                self.msg = Twist()
                self.angle_PID = PID(P=6, I=0, D=1)
                self.distance_PID = PID(P=0.7, I=0, D=0.1)

                # calculate the centre of robber turtle's probable path
                centre = self.calculate_robber_circle_centre(received_robber_pose)

                # calculate the radius of arc provided angular velocity exists
                if received_robber_pose.angular_velocity != 0:
                    radius = received_robber_pose.linear_velocity / received_robber_pose.angular_velocity

                    # 10% radius for error handling (could be tuned to be better)
                    radius = radius + radius*0.1

                # if centre is none, mainly because of zero angular velocity
                if centre == (None, None):
                    hyp = self.previous_co_ordinates_of_robber.linear_velocity * 5
                    self.goal_x = self.previous_co_ordinates_of_robber.x + hyp * math.cos(math.pi - self.previous_co_ordinates_of_robber.theta)
                    self.goal_y = self.previous_co_ordinates_of_robber.y + hyp * math.sin(math.pi - self.previous_co_ordinates_of_robber.theta)

                    print(f'PREDICTED: {self.goal_x, self.goal_y}')
                    # self.goal_x, self.goal_y = x2, y2

                else:
                    # define the time delta and time incrementer
                    time_delta = 0      # total time in which future co-ordinate will be approached
                    time_incrementer = 1        # by how much increments we should calculate the next co-ordinate

                    while True:
                        time_delta += time_incrementer

                        # predict tha angle after time_delta has passed (error adjustment for time-delta of 10%)
                        predicted_angle_wrt_centre = (received_robber_pose.angular_velocity * (time_delta + time_delta*0.1)) + received_robber_pose.theta - (math.pi / 2)
                        x2 = centre[0] + radius * math.cos(predicted_angle_wrt_centre)
                        y2 = centre[1] + radius * math.sin(predicted_angle_wrt_centre)

                        # check if it is reachable or not
                        distance_to_next_co_ordinate = self.euclidian_distance((police_pose_data.x, police_pose_data.y), (x2, y2))
                        if distance_to_next_co_ordinate < POLICE_TURTLE_SPEED*(time_delta - 0.01):
                            break

                    # set the goal
                    print(f'FUTURE: {x2, y2}')
                    self.goal_x, self.goal_y = x2, y2

                    chasing_key = True
                    expected_time_to_reach = time_delta
                    chase_start_time = time.time()

            # start movement of police turtle
            self.control_distance_new(POLICE_TURTLE_SPEED)
            self.control_angle_new()
            self.publish_turtle.publish(self.msg)

            # halt the police turtle if reached at latest received robber co-ordinates or
            if chasing_key:
                if self.distance < 0.5 and (len(self.previous_co_ordinates_of_robber) > 3):
                    chasing_key = False
                    self.publish_turtle.publish(self.stop_msg)
                    print('WAITING ...')

        print('FINISHED CHASE')

    def control_angle_new(self):
        target_angle = math.atan2(self.goal_y - police_pose_data.y, self.goal_x - police_pose_data.x)
        angle_difference = target_angle - police_pose_data.theta
        self.PID_angle = self.angle_PID.update(angle_difference)
        self.msg.angular.z = self.PID_angle

    def control_distance_new(self, max_vel):
        self.distance = math.sqrt((self.goal_x - police_pose_data.x) ** 2 + (self.goal_y - police_pose_data.y) ** 2)
        self.PID_distance = self.distance_PID.update(self.distance)
        if self.PID_distance > max_vel:
            self.msg.linear.x = max_vel

    @staticmethod
    def spawn():
        x = random.random() * 30
        y = random.random() * 20
        theta = random.random() * math.pi
        print(f"Waiting for 10 seconds before spawning PT!")
        rospy.wait_for_service('/spawn')
        time.sleep(1)
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
    global police_pose_data
    police_pose_data = data


def robber_pose_callback(data):
    # call back for position of robber turtle
    global robber_pose_data
    robber_pose_data = data


if __name__ == '__main__':
    try:
        # global variables for position and caught status of turtles
        police_pose_data = None
        rt_real_pose = None
        robber_pose_data = None
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
