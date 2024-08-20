# Assignments for Module #3 : Robot structure with URDF
- Create all files in *module_3_assignment* package


### Assignment 1: Robotic Arm Creation using TF
- **Tasks**:
Create a custom transform tree for a
    - Robotics arm of 3 DOF without body ( only transforms)
    - visualize it in rviz + utilize joint state publisher GUI to see transforms
    - You should not have any visualize tag filled yet.
### Assignment 2: Joints Understanding
- **Tasks**:
Add joints to the same Robotic Arm that you created earlier
    - Finger joints with prismatic joint type
    - Have base joint as continous
    - All other joints should be Revolute
    - Add visualize tag to your robot urdf and create body mostly using cylinders
### Assignment 3: Building Mobile Manupilator
- **Tasks**:
Put your robotic arm on top of different drive robot
    - Connect using base_link of diff bot.


### To run the code use following command
- **Rviz display node**
insted of keeping urdf name as module 3 name iskept as yodha_robot_urdf so its seperately added
all 3 tasks are combined and implemented using fusion 360 software.

1) ros2 launch yodha_robot_urdf_description display.launch.py
2) ros2 launch yodha_robot_urdf_description gazebo.launch.py 

