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
            std::vector<std::vector<double>> trajectory = loadTrajectory(file);

            // // std::string my_joint = "katana_motor2_lift_joint";
            // std::string my_joint = "planar_RR_joint1";
            // joint_ = hw->getHandle(my_joint);
            // command_ = joint_.getPosition();

            // gain_ = 100.0;
            
            sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &AdaptiveController::setCommandCB, this);
            ROS_INFO("Initialisation complete!");

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

        std::vector<std::vector<double>> loadTrajectory(std::istream& str)
        {
            const char delim = ',';
            std::vector<std::vector<double>> trajectory;
            std::string line;    

            while(std::getline(str,line))
            {
                std::vector<double> trajectory_pt;
                std::stringstream  ss(line);
                std::string cell;

                while (std::getline(ss, cell, delim)) 
                {
                    double cell_double = atof(cell.c_str());
                    trajectory_pt.push_back(cell_double);
                }
                trajectory.push_back(trajectory_pt);
            }

            return trajectory;
        }

        private:
            hardware_interface::JointHandle joint_;
            double gain_;
            double command_;
            ros::Subscriber sub_command_;

    };

    PLUGINLIB_EXPORT_CLASS(adaptive_controller_ns::AdaptiveController, controller_interface::ControllerBase);
}

