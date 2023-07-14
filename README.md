# Flytbase Assignments
Assignments submission for flytbase

Hello,
I am a beginner in RoS. I have tried my best to complete the goals provided accurately.

## Objectives: 

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

### Goal 2: Make the bot go on a given grid
- the code for this is present inside `follow_grid_limit_acceleration.py`
- 
[![Watch the video](videos/goal_2_follow_grid_with_limit_on_acceleration.gif)](videos/goal_2_follow_grid_with_limit_on_acceleration.gif)

### Goal 3: Move the bot in circle of given radius and speed
- the code for this is present inside `circle_publish_position.py`
- 
[![Watch the video](videos/goal_3_circular_path_publish_pose_and_noise.gif)](videos/goal_3_circular_path_publish_pose_and_noise.gif)



