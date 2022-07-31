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
            ROS_INFO("Trajectory points detected: %ld", trajectory.size());

            // Get the joint names
            std::vector<std::string> joint_names;
            if(!n.getParam("joints", joint_names) || joint_names.size() != 2)
            {
                ROS_ERROR("Failed to find the joint names OR num of joints != 2");
                return false;
            }

            // Get the joint handler and set the desired position
            for (size_t i = 0; i < joint_names.size(); i++)
            {
                joints_[i] = hw->getHandle(joint_names[i]);
                command_[i] = joints_[i].getPosition();
            }
            
            ROS_INFO("Initialisation complete!");
            prev_time = ros::Time::now();
            return true;
        }

        void update(const ros::Time& time, const ros::Duration& period)
        {
            ros::Time curr_time = ros::Time::now();
            ros::Duration duration = curr_time - prev_time;
            ROS_INFO("Elapsed sec: %f", duration.toSec());
            prev_time = ros::Time::now();
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
            hardware_interface::JointHandle joints_[2];
            double gain_;
            double command_[2];
            ros::Subscriber sub_command_;
            ros::Time prev_time;

    };

    PLUGINLIB_EXPORT_CLASS(adaptive_controller_ns::AdaptiveController, controller_interface::ControllerBase);
}

