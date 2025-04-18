# Learning ROS and Turtle Bot

---
### Catkin

- **create a workspace for catkin**  
 `mkdir catkin_ws`
- **create your package folder named 'turtlesim_cleaner_test'. (you can use any name if you want)**  
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
- in addition to functions in goal-1 I have added a function called `limit_linear_acceleration` and `start_grid`
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
### Goal 4: Chase robber turtle bot using another cop bot
- added threading for parallely control both the turtle bots as well update the positions using subscribers (listener)
- code for this goal is present inside `robber_cop_chase.py`
- `class RobberTurtle`
  - similar turtle as goal 3, only made to rotate in the circle of given radius and velocity
  - publishes its real positions and noisy positions on their respective topics.
  - will break the loop if it gets caught by PoliceTurtle
- `class PoliceTurtle`
  - `def start_chase()`
    - call the `spawn()` function to spawn the police turtle
    - will get the real pose published by robber every 5 seconds and assign as the current target
    - call the `control_angle_new()` and `control_distance_new()` functions to start the movement of police turtle to the received target
    - stop the police turtle momentarily if the old coordinates of robber turtle are reached
    - break the loop and stop the thread, once the distance between robber turtle and police turtle is below threshold
  - `control_angle_new()`
    - return the calculated PID value for angle control
  - `control_distance_new()`
    - return the calculated PID calue for distance control
  - `spawn()`
    - waits for 10 seconds and spawns police turtle at random position at random angle
- `listener()`
  - subscribe the ros topics for positions of turtle1, turtle2 and real_positions
  - will update on callback functions `robber_pose_callback()`, `robber_pose_callback()`, `robber_pose_callback()` respectively
  - along with `RobberTurtle.start_circle()` and `PoliceTurtle.start_chase()`, this function will also be on thread.

[![Watch the video](videos/goal_4_robber_cop_chase.gif)](videos/goal_4_robber_cop_chase.gif)

---
### Goal 5: Chase Robber turtle bot using Police turtle bot with half the velocity of robber-turtle
- for this goal, the planning is based on prediction of robber turtle's upcoming co-ordinates
- the prediction is calculated using the upcoming co-ordinates of robber turtle's the latest position, its angular velocity and the radius of circle it is actually travelling in
- this goal is coded inside the file `robber_slow_cop_planned_chase.py`
- `class RobberTurtle`
  - this class is very much same as in previous goal
- `class PoliceTurtle`
  - this class is almost same as in previous goal, except for following functions
  - `calculate_robber_circle()`
    - calculates the centre and radius of circle that will pass through given 3 points
    - If the centre is arbitrarily large value, or radius tends to infinity because the turtle is travelling sticking to the frame, I have assumed the centre as police turtle and radius as 100
  - `future_co_ordinates()`
    - calculates the future co-ordinates using current angle of robber turtle w.r.t centre, angle it will traverse in time 't'(t = 2xRxangle/V) we iterate these until we get the time which in which the Police Turtle will traverse to the point where Robber Turtle's next 'n'th co-ordinates will be.
  - `start_chase()`
    - this function will not start movement unless we calculate the centre of the circle
    - the maximum velocity which is half of robber's velocity is also used as a limit here

[![Watch the video](videos/goal_5_robber_slow_cop_planned_chase.gif)](videos/goal_5_robber_slow_cop_planned_chase.gif)

---
### Goal 6: Chase Robber turtle bot using Police turtle bot with half the velocity of robber-turtle and noisy co-ordinates of robber
- this goal is coded inside the file `robber_slow_cop_noisy_planned_chase.py`
- for this goal I implemented the same plan as of goal 5, only topics were renamed
- Although to calculate the centre I have waited for almost 6 co-ordinates to calculate the mean of noisy co-ordinates which will be closer to the actual co-ordinates of the centre.
- For larger radius like 30, it will fail because of the incrementing error while calculating the centre as well as predicting the future co-ordinates
- This objective is still completed, provided the standard deviation of the noise data of robber turtle is less than 2.
- If it starts getting higher, the co-ordinates will be un-predictable and hence cannot be planned

[![Watch the video](videos/goal_6_chase_noisy.gif)](videos/goal_6_chase_noisy.gif)


