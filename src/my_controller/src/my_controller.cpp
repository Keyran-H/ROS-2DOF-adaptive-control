#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>

namespace my_controller_ns
{

    class MyPositionController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
        bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
        {
            ROS_INFO("CP1");
            std::vector<std::string> joint_names;
            if(!n.getParam("joints", joint_names) || joint_names.size() != 2)
            {
                ROS_ERROR("Failed to find the joint names");
                return false;
            }
            // ROS_INFO("%s",joint_names[0].c_str());
            // ROS_INFO("%s",joint_names[1].c_str());
            for (size_t i = 0; i < joint_names.size(); i++)
            {
                joints_[i] = hw->getHandle(joint_names[i]);
                command_[i] = joints_[i].getPosition();
            }
            
            ROS_INFO("CP2");
            // std::string my_joint = "planar_RR_joint1";
            // joint_ = hw->getHandle(my_joint);
            // command_ = joint_.getPosition();

            gain_ = 0.1;
            
            sub_command_1 = n.subscribe<std_msgs::Float64>("command1", 1, &MyPositionController::setCommandCB1, this);
            sub_command_2 = n.subscribe<std_msgs::Float64>("command2", 1, &MyPositionController::setCommandCB2, this);

            return true;
        }

        void update(const ros::Time& time, const ros::Duration& period)
        {
            double tol = 0.5;
            double error_1 = command_[0] - joints_[0].getPosition();
            double error_2 = command_[1] - joints_[1].getPosition();
            double commanded_effort_1 = error_1 * (gain_*100);
            double commanded_effort_2 = error_2 * gain_;

            // Mini-algorithm: Essentially, wait until the first joint reaches setpoint for 5 seconds before correcting second joint. 

            if(fabs(error_1) < tol) 
            {
                ROS_INFO("Elapsed: %f", ros::Time::now().toSec() - now.toSec());
                if (ros::Time::now().toSec() - now.toSec() > 5.0)
                {
                    ROS_INFO("commanded_effort_2: %f", commanded_effort_2);
                    ROS_INFO("Joint2 Position: %f", joints_[1].getPosition());
                    joints_[1].setCommand(commanded_effort_2);
                }
            }
            else
            {
                now = ros::Time::now();
                ROS_INFO("commanded_effort_1: %f", commanded_effort_1);
                ROS_INFO("Joint1 Position: %f", joints_[0].getPosition());
                joints_[0].setCommand(commanded_effort_1);
            }
        }

        void setCommandCB1(const std_msgs::Float64ConstPtr& msg)
        {
            command_[0] = msg->data;
        }

        void setCommandCB2(const std_msgs::Float64ConstPtr& msg)
        {
            command_[1] = msg->data;
        }

        void starting(const ros::Time& time) { }
        void stopping(const ros::Time& time) { }

        private:
            hardware_interface::JointHandle joints_[2];
            double gain_;
            double command_[2];
            ros::Subscriber sub_command_1;
            ros::Subscriber sub_command_2;
            ros::Time now;

    };

    PLUGINLIB_EXPORT_CLASS(my_controller_ns::MyPositionController, controller_interface::ControllerBase);
}