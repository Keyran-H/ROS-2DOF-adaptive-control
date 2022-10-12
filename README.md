# Adaptive Control of a 2DOF Planar Robot

This repository comprises two ROS packages, each containing a custom ros_control controller example and a folder comprising matlab scripts used for processing and visualising data. The ros_control ROS packages are:
- adaptive_controller: This is used to simulate a torque controlled 2DOF Planar robot in Gazebo using a gradient descent adaptive controller.
- novel_adaptive_controller_paper: This is used to simulate a torque controlled 2DOF humanoid robot arm in Gazebo using the novel adaptive controller from [this](https://www.sciencedirect.com/science/article/abs/pii/S0921889013001887) paper.

# Running the simulations

Use the planar_RR_robot_sim.launch to run the simulation for the gradient descent adaptive controller:

`roslaunch adaptive_controller planar_RR_robot_sim.launch`

Use the BERT2_sim.launch to run the simulation for the novel adaptive controller:

`roslaunch novel_adaptive_controller_paper BERT2_sim.launch`

The parameters for the simulation can be adjusted using the yaml file in the config folder. Here is an example of the simulation with the adaptive_controller ROS package.

![](https://github.com/Keyran-H/src/gif/GradientDescentPlanarRobot.gif)

# Miscellaneous

Software Versions:
- ROS: melodic
- Gazebo: 9.0.0

Useful Resources:
- [How to create your own controller (Video)](https://www.youtube.com/watch?v=7BLc18lOFJw)
- [franks_ros custom controller examples](https://github.com/frankaemika/franka_ros/tree/develop/franka_example_controllers/src)
