#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include "ros/ros.h"

namespace controller_ns{

class PositionController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  PositionController(){}
  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n) // I don't know how to use the hardware_interface code.
  {
    // get joint name from the parameter server
    std::string my_joint = "katana_motor2_lift_joint"; // Start Gazebo first!!!!!!!!
    // if (!n.getParam("joint", my_joint)){
    //   ROS_ERROR("Could not find joint name");
    //   return false;
    // }

    // get the joint object to use in the realtime loop
    joint_ = hw->getHandle(my_joint);  // throws on failure
    return true;
  }

  void update(const ros::Time& time, const ros::Duration& period)
  {
    double error = setpoint_ - joint_.getPosition(); // I think
    joint_.setCommand(error*gain_);
  }

  void starting(const ros::Time& time) { }
  void stopping(const ros::Time& time) { }

private:
  hardware_interface::JointHandle joint_;
  const double gain_ = 1.25;
  const double setpoint_ = 3.00;
};
// PLUGINLIB_DECLARE_CLASS(package_name, PositionController, controller_ns::PositionController, controller_interface::ControllerBase);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "controller");
    controller_ns::PositionController ctrl;


    while (ros::ok()){
        const ros::Time time = ros::Time::now();
        const ros::Duration period = time - prev_time;

        robot.read();
        cm.update(time, period);
        robot.write();
        
        rate.sleep();
    }
    
    // ros::spin();

}