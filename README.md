# Adaptive Control of a 2DOF Planar Robot

This repository comprises two ROS packages, each containing a custom ros_control example. The repository also contains a folder which comprise a bunch of matlab scripts used for processing and visualising data. The ros_control ROS packages are:
- adaptive_controller: This is used to simulate a torque control 2DOF robot in Gazebo using a gradient descent adaptive controller
- novel_adaptive_controller_paper: This is used to simulate a torque control 2DOF robot in Gazebo using a novel adaptive controller

# Running the simulations

Use the planar_RR_robot_sim.launch to run the simulation for the gradient descent adaptive controller:

'roslaunch adaptive_controller planar_RR_robot_sim.launch'

Use the BERT2_sim.launch to run the simulation for the novel adaptive controller:

'roslaunch novel_adaptive_controller_paper BERT2_sim.launch'

The parameters for the simulation can be adjusted using the yaml file in the config folder.

# Miscellaneous

Software Versions:
- ROS: melodic
- Gazebo: Version 9

Useful Resources:
- 
