# Assignments for Module #4 : Robot Sensing
- Create all files in *module_4_assignment* package

# ----------------------------------------------------------------------------
### Task 1: restructure line following code
### Ans = 
1) ros2 launch module_4_assignment module_4_camera_lineFollowing.launch.py

# ----------------------------------------------------------------------------

### Task 2: Design Software Flow for maze Solving
in the folder module_4_assessment->Diagram->software flow for maze solving.

# ----------------------------------------------------------------------------

### Task 3: Implement Maze Solving in Gazebo
- **Tasks**:
Create a maze in gazebo to perform wall following
    - Detect Wall on right with Lidar Sensor
    - Should create an envionrment of sqaure shape
    - Robot will keep the structure on its right
### Ans =
1) ros2 launch module_4_assignment module_4_lidar_wallFollowing.launch.py
    Run this command to run robot wall following mode
2) robot did wall following using PID , so it is able to do Left side and Right side wall.
    to chnage the side make sure setting is like
    
    1) rightFlag = 1, leftFlag = 0; // this will start following right side wall
    2) rightFlag = 0, leftFlag = 1; // this will start following left side wall
    3) Set wallFollowingDistance veriable set distance as you want. (currently its 3 meter)
3)  if the wall following shifted from more than 2 meters from its current possition it will stop.
4) when front wall is near it will reduce its speed by 60% till it take turn.

# ----------------------------------------------------------------------------

### Task 3: Calculate Robot Speed
- **Tasks**:
Utilize Turltebo3 IMU sensor to calculate robot
    - Velocity
    - Acceleration
### Ans = 
commands to use 
1) ros2 launch bal_robot_urdf_description gazebo.launch.py
2) ros2 run module_4_assignment imu_data_process 
3) ros2 run teleop_twist_keyboard teleop_twist_keyboard to move the robot 
 // to check the velocity and acceleration of the robot.also use teleop twist keyboard
# ----------------------------------------------------------------------------

