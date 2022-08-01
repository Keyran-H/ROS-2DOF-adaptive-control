#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>
#include <fstream>
#include <Eigen/Core>

// % Initial Estimates
#define m1 2
#define m2 2
#define l1 2
#define l2 2
#define Izz1 2
#define Izz2 2

namespace adaptive_controller_ns
{
    class AdaptiveController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
        bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
        {
            // Read from the csv file
            std::ifstream file("/home/kiran/dissertation/ros_experimenting_ws/src/matlab_files/trajectory_test.csv");
            trajectory = loadTrajectory(file);
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

            // Eigen::MatrixXd theta_hat(5,1);
            theta_hat(0,0) = m1*pow(l1,2) + m2*pow(l1,2) + m2*pow(l2,2) + Izz1 + Izz2;
            theta_hat(1,0) = m2*l1*l2;
            theta_hat(2,0) = m2*pow(l2,2) + Izz2;
            theta_hat(3,0) = m1*l1 + m2*l1;
            theta_hat(4,0) = m2*l2;

            // std::cout  << "(" << theta_hat.rows() << ", " << theta_hat.cols() << ")";
            // std::cout << "\n" << theta_hat << "\n";

            return true;
        }

        // TODO: Figure out exactly what these arguments mean
        void update(const ros::Time& time, const ros::Duration& period)
        {

            
            ros::Time curr_time = ros::Time::now();
            ros::Duration duration = time - prev_time;
            // ROS_INFO("Elapsed sec: %f", duration.toSec());
            // ROS_INFO("Elapsed sec: %f", duration.toSec());
            
            // Compute errors
            
            
            
            prev_time = ros::Time::now();
        }

        void starting(const ros::Time& time) { }
        void stopping(const ros::Time& time) { }

        void getPhi()
        {
            // Make the state variables
            Eigen::MatrixXd q_robot(2,1);
            q_robot(0,0) = joints_[1].getPosition();
            q_robot(1,0) = joints_[2].getPosition();

            Eigen::MatrixXd qd_robot(2,1);
            qd_robot(0,0) = joints_[1].getVelocity();
            qd_robot(1,0) = joints_[2].getVelocity();

            
            Eigen::MatrixXd Phi(2,5);
            Phi(0,1) = 1;
            Phi(0,2) = 1;
            Phi(0,3) = 1;
            Phi(0,4) = 1;
            Phi(0,5) = 1;
            Phi(1,1) = 1;
            Phi(1,2) = 1;
            Phi(1,3) = 1;
            Phi(1,4) = 1;
            Phi(1,5) = 1; 
            return;
        }
        
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

        public:
            AdaptiveController() : theta_hat(5,1){ 
                
            }
        
        private:
            hardware_interface::JointHandle joints_[2];
            double gain_;
            double command_[2];
            ros::Subscriber sub_command_;
            ros::Time prev_time;
            std::vector<std::vector<double>> trajectory;
            Eigen::MatrixXd theta_hat;
            unsigned int traj_idx = 0;

    };

    PLUGINLIB_EXPORT_CLASS(adaptive_controller_ns::AdaptiveController, controller_interface::ControllerBase);
}

