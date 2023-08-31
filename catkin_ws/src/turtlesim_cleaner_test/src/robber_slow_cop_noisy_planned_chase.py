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
RADIUS = 10
PT_SPAWN_TIME = 10
STD_DEVIATION = 3


class RobberTurtle:
    def __init__(self):
        # publishing topics
        self.publish_turtle = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)  # turtle commands
        self.pos_publish_real = rospy.Publisher("/rt_real_pose", Pose, queue_size=10)  # turtle real pose
        self.pos_publish_noise = rospy.Publisher("/rt_noisy_pose", Pose, queue_size=10)  # turtle noisy pose

        # define twist object for publishing actions
        self.msg = Twist()
        self.noise = np.random.normal(0, STD_DEVIATION, 100)

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

        _pn.x = self.current_pose.x + random.choice(self.noise)
        _pn.y = self.current_pose.y + random.choice(self.noise)
        _pn.theta = self.current_pose.theta + random.choice(self.noise)
        _pn.angular_velocity = self.current_pose.angular_velocity + random.choice(self.noise)
        _pn.linear_velocity = self.current_pose.linear_velocity + random.choice(self.noise)

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

        self.previous_centres_x = []
        self.previous_centres_y = []
        self.previous_radii = []

        self.calculated_radius_list = []
        self.calculated_centre_x_list = []
        self.calculated_centre_y_list = []

        # define twist object for publishing actions
        self.msg = Twist()
        self.distance = 100
        self.stop_msg = Twist()
        self.stop_msg.linear.x = 0
        self.stop_msg.angular.z = 0

    @staticmethod
    def calculate_robber_circle_centre(robber_pose):
        """
        Unused in goal 6, used in goal 5
        calculates the centre of robber circle, using current Pose() of robber
        :param robber_pose: robber Pose()
        :return: centre, radius
        """
        cx = None
        cy = None
        radius = None
        if robber_pose.angular_velocity != 0:
            radius = abs(robber_pose.linear_velocity / robber_pose.angular_velocity)
            # 10% radius for error handling (could be tuned to be better)
            radius = radius + radius * 0.1
            cx = robber_pose.x - radius * math.cos(robber_pose.theta - math.pi/2)
            cy = robber_pose.y - radius * math.sin(robber_pose.theta - math.pi/2)
        return (cx, cy), radius

    def calculate_robber_circle(self, points_list):
        """
        calculates the circum circle of triangle formed by recent 3 values of robber turtle
        :param points_list: list of last 3 coordinates of robber circle
        :return: centre, radius
        """
        radius = 0
        points_list = points_list[-3:]

        A = points_list[0]
        B = points_list[1]
        C = points_list[2]
        D = 2*(A.x*(B.y - C.y) + B.x*(C.y - A.y) + C.x*(A.y - B.y))
        Ux = (A.x**2 + A.y**2)*(B.y - C.y) + (B.x**2 + B.y**2)*(C.y - A.y) + (C.x**2 + C.y**2)*(A.y - B.y)
        Uy = (A.x**2 + A.y**2)*(C.x - B.x) + (B.x**2 + B.y**2)*(A.x - C.x) + (C.x**2 + C.y**2)*(B.x - A.x)

        if D != 0:
            # cn += 1
            centre = (Ux/D, Uy/D)
            radius_a = self.euclidian_distance(centre, (A.x, A.y))
            radius_b = self.euclidian_distance(centre, (B.x, B.y))
            radius_c = self.euclidian_distance(centre, (C.x, C.y))
            radius += (radius_a + radius_b + radius_c)/3
        else:
            centre = (None, None)
            radius = None
        radius = radius

        print(f'travelling on a circle : {centre} | {radius}')

        return centre, radius

    def calculate_simple_mean_robber_circle_centre(self, robber_pose):
        """
        calculates the mean value of robber's path arc radius and calculates the centre
        :param robber_pose: Pose() of robber turtle
        :return: (centre cx, centre cy), mean radius
        """
        cx = None
        cy = None
        radius = None
        if robber_pose.angular_velocity != 0:
            radius = abs(robber_pose.linear_velocity / robber_pose.angular_velocity)
            self.previous_radii.append(radius)
            radius = sum(self.previous_radii) / len(self.previous_radii)
            # 10% radius for error handling (could be tuned to be better)
            # radius = radius + radius * 0.1
            cx = robber_pose.x - radius * math.cos(robber_pose.theta - math.pi/2)
            cy = robber_pose.y - radius * math.sin(robber_pose.theta - math.pi/2)
        print(f'simple mean circle : {(cx, cy)} | {radius}')

        return (cx, cy), radius

    def calculate_filtered_robber_circle_centre(self, robber_pose):
        """
        Failed Function: Kept for Reference
        This is an unused function, it basically calculates the median value from all the previous values of radius and
        centre's coordinates
        :param robber_pose: Pose() object of robber
        :return: filtered centre, filtered radius
        """
        filtered_cx = None
        filtered_cy = None
        radius = None
        if robber_pose.angular_velocity != 0:
            radius = abs(robber_pose.linear_velocity / robber_pose.angular_velocity)
            self.previous_radii = self.insert_in_sorted_list(self.previous_radii, radius)
            filtered_radii = self.median_filter(self.previous_radii)
            print(f'\n----------------------------radii------------------------\n'
                  f'{radius} | {filtered_radii}\n'
                  f'{sum(self.previous_radii)/len(self.previous_radii)}\n'
                  f'{self.previous_radii}\n'
                  f'-------------------------------\n')
            # 10% radius for error handling (could be tuned to be better)
            radius = filtered_radii + filtered_radii * 0.1
            cx = robber_pose.x - radius * math.cos(robber_pose.theta - math.pi/2)
            self.previous_centres_x = self.insert_in_sorted_list(self.previous_centres_x, cx)
            filtered_cx = self.median_filter(self.previous_centres_x)

            cy = robber_pose.y - radius * math.sin(robber_pose.theta - math.pi/2)
            self.previous_centres_y = self.insert_in_sorted_list(self.previous_centres_y, cy)
            filtered_cy = self.median_filter(self.previous_centres_y)
            print(f'\n-----------------------centres------------------------\n'
                  f'({cx, cy}) | ({filtered_cx, filtered_cy})\n'
                  f'{self.previous_centres_x}\n'
                  f'{self.previous_centres_y}\n'
                  f'-------------------------------\n')

        return (filtered_cx, filtered_cy), filtered_radii

    @staticmethod
    def future_co_ordinates(centre, radius, current_robber_loc, previous_robber_loc):
        """
        Working function, but unused as I have better logic right now. Kept for future references
        :param centre: centre of the arc robber is traversing in
        :param radius: radius of arc of the robber's path
        :param current_robber_loc: recently received robber coordinates
        :param previous_robber_loc: previous received robber coordinates
        :return: predicted Pose()
        """
        future_pose = Pose()
        current_angle = math.atan2(current_robber_loc.y - centre[1], current_robber_loc.x - centre[0])
        previous_angle = math.atan2(previous_robber_loc.y - centre[1], previous_robber_loc.x - centre[0])

        angle_difference = math.degrees(current_angle - previous_angle)
        future_angle = math.degrees(current_angle) + angle_difference

        if future_angle < 0:
            future_angle = future_angle + 360

        future_pose.x, future_pose.y = (centre[0] + radius * math.cos(math.radians(future_angle)), centre[1] + radius * math.sin(math.radians(future_angle)))
        return future_pose

    @staticmethod
    def euclidian_distance(A, B):
        """
        returns euclidean distance between 2 points
        :param A: Pose() of one point
        :param B: Pose() of seconds point
        :return: distance
        """
        return math.sqrt((A[0] - B[0])**2 + (A[1] - B[1])**2)

    @staticmethod
    def get_new_co_ordinates_of_robber(start_time):
        """
        returns coordinates of robber if 5s have passed
        :param start_time: start time of the instance when previous co-ordinates of robber were received
        :return: Pose() of robber
        """
        _ = robber_turtle_noise_pose

        if time.time() - start_time >= 5:
            print(f'\nNEW NOISY CO_ORDINATES RECEIVED ----\n{_}\n----------------\n')
            print(f'\nNEW REAL CO_ORDINATES RECEIVED ----\n{robber_turtle_real_pose}\n----------------\n')
            return _
        else:
            return False

    @staticmethod
    def insert_in_sorted_list(previous_values_list, n):
        """
        Inserts a value in a sorted list, without disturbing its sorting
        :param previous_values_list: list of values
        :param n: value needed to be added
        :return: list of values with new value added
        """
        index = len(previous_values_list)
        # Searching for the position
        for i in range(len(previous_values_list)):
            if previous_values_list[i] > n:
                index = i
                break
        # Inserting n in the list
        if index == len(previous_values_list):
            previous_values_list = previous_values_list[:index] + [n]
        else:
            previous_values_list = previous_values_list[:index] + [n] + previous_values_list[index:]
        return previous_values_list

    @staticmethod
    def median_filter(list_of_noisy_values):
        """
        calculates the median of a sorted list
        :param list_of_noisy_values: sorted list of values
        :return: median of the list
        """
        mid = len(list_of_noisy_values) // 2
        median_of_values = (list_of_noisy_values[mid] + list_of_noisy_values[~mid]) / 2
        return median_of_values

    @staticmethod
    def check_if_turtle_stuck_at_edge(previous_co_ordinates_of_police, current_co_ordinates_of_police):
        """
        validates if turtle is tuck on the edge of the screen
        :param previous_co_ordinates_of_police: previous Pose of turtle
        :param current_co_ordinates_of_police: current Pose of turtle
        :return:
        """
        if previous_co_ordinates_of_police is None or current_co_ordinates_of_police is None:
            return False
        if (abs(previous_co_ordinates_of_police.x - current_co_ordinates_of_police.x) < 0.01) or \
                (abs(previous_co_ordinates_of_police.y - current_co_ordinates_of_police.y) < 0.01):
            return True
        else:
            return False

    def start_chase(self):
        """
        starts the main chase
        :return: None
        """

        # spawn the police turtle
        self.spawn()

        # initial goal parameters
        self.goal_x = 5.45
        self.goal_y = 5.45

        # start time defined
        start_time = time.time()

        chasing_key = False

        # time related variables
        expected_time_to_reach = 0
        time_elapsed = 0

        centre = (None, None)

        # turtle pose values
        previous_police_turtle_pose = None
        recieved_robber_pose = None
        filtered_robber_pose = None
        recieved_pose_counter = 0
        while not rospy.is_shutdown():

            # initialize global variables
            global police_pose_data, robber_pose_data, caught

            # fetch the new co-ordinates if available
            _received = self.get_new_co_ordinates_of_robber(start_time)

            # if fetched start with controlling
            if _received:
                recieved_pose_counter += 1

                # save the robber turtle pose
                recieved_robber_pose = _received
                start_time = time.time()

                # append to the list of previous co-ordinates
                self.previous_co_ordinates_of_robber.append(recieved_robber_pose)
                filtered_robber_pose = recieved_robber_pose
                _received = False

                # calculate the centre of robber turtle's probable path

                # calculate centre and radius of arc
                if len(self.previous_co_ordinates_of_robber) > 3:
                    # calculate median of radius values
                    centre, radius = self.calculate_robber_circle(self.previous_co_ordinates_of_robber)
                    self.calculated_radius_list = self.insert_in_sorted_list(self.calculated_radius_list, radius)
                    radius = self.median_filter(self.calculated_radius_list)

                    # calculate median of centre's x-coordinate values
                    self.calculated_centre_x_list = self.insert_in_sorted_list(self.calculated_centre_x_list, centre[0])
                    cx = self.median_filter(self.calculated_centre_x_list)

                    # calculate median of centre's y-coordinate values
                    self.calculated_centre_y_list = self.insert_in_sorted_list(self.calculated_centre_y_list, centre[1])
                    cy = self.median_filter(self.calculated_centre_y_list)

                    centre = (cx, cy)
                    print(f'FINAL RADIUS: {radius} | {centre}')
                else:
                    centre, radius = self.calculate_simple_mean_robber_circle_centre(filtered_robber_pose)

                time_elapsed += 5

                # checking if cop has reached the edge
                if self.check_if_turtle_stuck_at_edge(police_pose_data, previous_police_turtle_pose):
                    chasing_key = False
                    print('POLICE STUCK ON EDGE')

                previous_police_turtle_pose = police_pose_data

            # start the movement when subscribed data is received
            if police_pose_data is None or filtered_robber_pose is None:
                continue

            # if distance between cop and robber is less than threshold, COMPLETED CHASE
            if self.euclidian_distance(
                    (police_pose_data.x, police_pose_data.y),
                    (robber_pose_data.x, robber_pose_data.y)) < self.threshold_distance:
                caught = True
                self.publish_turtle.publish(self.stop_msg)

                print(f'CAUGHT\n{robber_pose_data}')
                break

            # if chasing has not started
            if not chasing_key:

                # initialize control variables of PID
                self.msg = Twist()
                self.angle_PID = PID(P=6, I=0, D=1)
                self.distance_PID = PID(P=0.7, I=0, D=0.1)

                # if centre is none, mainly because of zero angular velocity
                if centre == (None, None):
                    hyp = self.previous_co_ordinates_of_robber[-1].linear_velocity * 5
                    self.goal_x = self.previous_co_ordinates_of_robber[-1].x + hyp * math.cos(math.pi - self.previous_co_ordinates_of_robber[-1].theta)
                    self.goal_y = self.previous_co_ordinates_of_robber[-1].y + hyp * math.sin(math.pi - self.previous_co_ordinates_of_robber[-1].theta)

                    print(f'PREDICTED: {self.goal_x, self.goal_y}')
                else:
                    # define the time delta and time incrementer
                    time_delta = 0              # total time in which future co-ordinate will be approached
                    time_incrementer = 1        # by how much increments we should calculate the next co-ordinate

                    while True:
                        time_delta += time_incrementer

                        # predict tha angle after time_delta has passed (error adjustment for time-delta of 10%)
                        predicted_angle_wrt_centre = (filtered_robber_pose.angular_velocity * (time_delta + time_delta*0)) + filtered_robber_pose.theta - (math.pi / 2)
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

            # start movement of police turtle
            self.control_distance_new(POLICE_TURTLE_SPEED)
            self.control_angle_new()
            self.publish_turtle.publish(self.msg)

            # halt the police turtle if reached at latest received robber co-ordinates or
            if chasing_key:
                if self.distance < 0.5:
                    print(f'\npolice data\n{police_pose_data}\n---------\n')
                    chasing_key = False
                    # self.publish_turtle.publish(self.stop_msg)
                    self.msg = Twist()
                    self.angle_PID = PID(P=6, I=0, D=1)
                    self.distance_PID = PID(P=0.7, I=0, D=0.1)
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
        print(f"Waiting for {PT_SPAWN_TIME} seconds before spawning PT!")
        rospy.wait_for_service('/spawn')
        time.sleep(PT_SPAWN_TIME)
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
    rospy.Subscriber("/rt_noisy_pose", Pose, robber_noise_callback)
    rospy.spin()


def robber_real_callback(data):
    # call back for position of robber turtles which is updated every 5 seconds
    global robber_turtle_real_pose
    robber_turtle_real_pose = data


def robber_noise_callback(data):
    # call back for position of robber turtles which is updated every 5 seconds
    global robber_turtle_noise_pose
    robber_turtle_noise_pose = data


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
        robber_turtle_noise_pose = None
        robber_turtle_real_pose = None
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
