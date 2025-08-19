# INTRODUCTION
This project implements obstacle avoidance and goal-directed navigation for a differential drive robot in ROS 2 Jazzy using the Artificial Potential Fields (APF) algorithm.

The robot is simulated in Gazebo Harmonic, equipped with a LiDAR sensor, and controlled via a custom APF-based controller node.

# USAGE 
## Clone the repository 
- cd ~/ros2_ws/src
- git clone https://github.com/BlackkBeardd/apf_obstacle_avoidance.git
- cd ..
- colcon build
- source install/setup.bash

## Launch gazebo with robot 
ros2 launch diff_drive_description gazebo.launch.py

## Start the apf controller
ros2 launch diff_drive_control apf.launch.py

## Tuning and goal position
Set the goal position and tune the parameters in config/apf.yaml


# LIMITATIONS
- The robot sucessfully avoids obstacles using the artificial potential fields. 
- However, the robot does not converge to the goal point. It keeps moving around the obstacles without stopping at the target. 
- This is a known limitation of the APF approach where the robot gets stuck in the local minima. 

# POSSIBLE IMPROVEMENTS  
- Combine APF with A* planner
