# Flytbase Assignments

## Assignments submission for flytbase

---

> Hello,  
I am a beginner in RoS. I have tried my best to complete the goals provided accurately. I faced many challenges during installation and setup of this project.
I have used ROS-noetic in this project.  
    There are some flaws in the movement of the bot, eg: the bot in goal 1 and goal 2 is first rotating to the direction and then moving towards it.
In later parts i.e. during goal 4, I required a bot that would do rotating and moving simultaneously.
For goal 2 the approach where you rotate first and then move towards the objective suits well for the sharp turns on the grid.
I could not finish the goal 5 and goal 6 as I had already reached my deadline. I had work after my office hours everyday hence I could not give my 100% here.

## Installation

---
### Catkin

- **create a workspace for catkin**  
 `mkdir catkin_ws`
- **create a your package folder named 'turtlesim_cleaner_test'. (you can use any name if you want)**  
`catkin_create_pkg turtlesim_cleaner_test geometry_msgs rospy`
- **go back to main catkin_ws folder and build the project**  
`catkin_make`
- **add or create the python files inside this folder**  
`catkin_make/src/<package_name>/src/`
- **make the python files executable**  
`chmod +x <python_file_path>`
- **add the package to PACKAGE_PATH**  
`export ROS_PACKAGE_PATH=/opt/ros/noetic/share:/home/<user_name>/catkin_ws/src`

## Start

---
- **start the roscore**  
 `roscore`
- **start the turtlesim**  
`rosrun turtlesimnode turtlesim`  
- **run your package**  
`rosrun <package_name> <python_file_name>.py`  

## Objectives: 

---
### Goal 1: Movement of Turtle-bot wherever it is given by user
- the code for this is present inside `control_turtle_bot.py`
- input: X and Y co-ordinates of the destination
- utilizes PID control to reach the destination accurately without any overshoot
- `def control_angle()`
  - calculates the angle required to rotate to face the target.
  - calculates the accurate angular velocity required to achieve that rotation
- `def control_distance()`
  - calculates the distance between current position and target
  - calculates the accurate linear velocity required to reach the target
- `def get_user_input()`
  - takes user input : co-ordinate: X and co-ordinate: Y
- `def move_turtle()`
  - calls the `get_user_input`, `control_angle` and `control_distance` functions one after the other
- `def pose_callback()`
  - gets the current position of the turtle bot by subscribing to its topic and save it
  
[![Watch the video](videos/goal_1_control_turtle.gif)](videos/goal_1_control_turtle.gif)

---
### Goal 2: Make the bot go on a given grid
- the code for this is present inside `follow_grid_limit_acceleration.py`
- for this code I have made sure that the bot first turns towards the objective and then moves towards it.
- on addition to functions in goal1 I have added a function called `limit_linear_acceleration` and `start_grid`
- `def limit_linear_acceleration()`
  - a function that would take initial and final velocity along with initial time to compute acceleration and then compute the most close final velocity that could be published so that acceleration is not crossing the limit
  - In the video you can see the acceleration (velocity derivative) graph is getting limited to 100, the extra spikes can be handled by introducing a factor to the equation
- `def start_grid()`
  - A function that assigns the new co-ordinate of the grid upon reaching the current target co-ordinate.

[![Watch the video](videos/goal_2_follow_grid_with_limit_on_acceleration.gif)](videos/goal_2_follow_grid_with_limit_on_acceleration.gif)

---
### Goal 3: Move the bot in circle of given radius and speed
- the code for this is present inside `circle_publish_position.py`
- `def start circle()`
  - using the tangential velocity and radius we can compute the angular velocity required
  - acceleration limiter is also used in this function
- `def gaussian_noise()`
  - create gaussian noise using the np.random.normal()
  - here we use the actual value as mean and 10 as the standard deviation of noise

[![Watch the video](videos/goal_3_circular_path_publish_pose_and_noise.gif)](videos/goal_3_circular_path_publish_pose_and_noise.gif)

---
### Goal4: Chase robber turtle bot using another cop bot
- I failed in solving this problem statement, the main reason was subscribing to multiple topics.
- I tried `TimeSynchronizer` as well as `ApproximateTimeSynchronizer` but both failed
- using `ApproximateTimeSynchronizer` I was able to subscribe the topics, but they were not synchronous.
- the callback would update the positions/data only once
- I tried researching a lot of resources, but I could resolve my errors
- Since I could not finish this goal and I am very close to my deadline I could not complete `Goal5` and `Goal6`


