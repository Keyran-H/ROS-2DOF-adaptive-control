# Adaptive Control of a 2DOF Planar Robot

The repository comprises two ROS packages, each using a custom ros_control controller, and a folder containing matlab scripts used for processing and visualising data. The ROS packages are:
- *adaptive_controller*: Used to simulate a torque controlled 2DOF Planar robot in Gazebo with a gradient descent adaptive controller.
- *novel_adaptive_controller_paper*: Used to simulate a torque controlled 2DOF humanoid robot arm in Gazebo with a novel adaptive controller from [this](https://www.sciencedirect.com/science/article/abs/pii/S0921889013001887) paper.

# Running the simulations

Use "planar_RR_robot_sim.launch" to run the simulation from the *adaptive_controller* ROS Package:

`roslaunch adaptive_controller planar_RR_robot_sim.launch`

Use "BERT2_sim.launch" to run the simulation from the *novel_adaptive_controller_paper* ROS Package:

`roslaunch novel_adaptive_controller_paper BERT2_sim.launch`

The controller parameters for the simulation can be adjusted using the yaml file from the config folder for the respective ROS package. The GIF shows the simulation from the *adaptive_controller* ROS package.

<p align="center">
    <img src="https://github.com/Keyran-H/ros_experimenting_ws/blob/main/src/gif/GradientDescentPlanarRobot.gif" width="30%" height="30%"/>
</p>

# Miscellaneous

Software Version:
- ROS: melodic
- Gazebo: 9.0.0

Useful Resources:
- [How to create your own controller (~1hr Video)](https://www.youtube.com/watch?v=7BLc18lOFJw)
- [franka_ros custom controller examples](https://github.com/frankaemika/franka_ros/tree/develop/franka_example_controllers/src)
