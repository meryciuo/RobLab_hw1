## Robotics Lab Homework1
In order to run the three ROS packages, follow this instruction

##Build and Source the packages 

Clone the repository in your ROS2 workspace and launch those commands to build it:

colcon build 
source install/setup.bash

##Visualize the manipulator in RViz:
ros2 launch arm_description display_launch.py

## Visualize the manipulator in Gazebo with the controllers loaded
ros2 launch arm_gazebo arm_gazebo.launch.py


## Visualize the image seen by the camera
Open another terminal and run the command
rqt
then open the plugin rqt_image_view


## Run the ROS publisher and subscriber node
Open another terminal and run the command
ros2 run arm_control arm_controller

To modify the Joint position values run the command 
ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray "data: [j0_value, j1_value, j2_value, j3_value]"

