#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>
#include <fstream>

namespace adaptive_controller_ns
{
    class AdaptiveController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
        bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
        {

            // Read from the csv file
            std::ifstream file("/home/kiran/dissertation/ros_experimenting_ws/src/matlab_files/trajectory_test.csv");
            std::vector<std::string> data = AdaptiveController::getNextLineAndSplitIntoTokens(file);
            ROS_INFO("You got through, congrats!");

            // // std::string my_joint = "katana_motor2_lift_joint";
            // std::string my_joint = "planar_RR_joint1";
            // joint_ = hw->getHandle(my_joint);
            // command_ = joint_.getPosition();

            // gain_ = 100.0;
            
            sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &AdaptiveController::setCommandCB, this);

            return true;
        }

        void update(const ros::Time& time, const ros::Duration& period)
        {
            // double error = command_ - joint_.getPosition();
            // double commanded_effort = error * gain_;
            // ROS_INFO("commanded_effort: %f", commanded_effort);
            // ROS_INFO("Joint Position: %f", joint_.getPosition());
            // joint_.setCommand(commanded_effort);
        }

        void setCommandCB(const std_msgs::Float64ConstPtr& msg)
        {
            command_ = msg->data;
        }

        void starting(const ros::Time& time) { }
        void stopping(const ros::Time& time) { }

        std::vector<std::string> getNextLineAndSplitIntoTokens(std::istream& str)
        {
            std::vector<std::string>   result;
            std::string                line;
            std::getline(str,line);

            std::stringstream          lineStream(line);
            std::string                cell;

            while(std::getline(lineStream,cell, ','))
            {
                result.push_back(cell);
            }
            // This checks for a trailing comma with no data after it.
            if (!lineStream && cell.empty())
            {
                // If there was a trailing comma then add an empty element.
                result.push_back("");
            }
            return result;
        }

        private:
            hardware_interface::JointHandle joint_;
            double gain_;
            double command_;
            ros::Subscriber sub_command_;

    };

    PLUGINLIB_EXPORT_CLASS(adaptive_controller_ns::AdaptiveController, controller_interface::ControllerBase);
}

