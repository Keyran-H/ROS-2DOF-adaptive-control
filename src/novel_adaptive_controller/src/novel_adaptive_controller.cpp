#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>
#include <fstream>
#include <Eigen/Core>

// Initial Estimates
#define m1 2.4
#define m2 2.9
#define l1 0.3
#define l2 0.4
#define Izz1 0.002
#define Izz2 0.004

/*
TO launch this controller
    roslaunch ros_experimenting sim.launch
    rosrun gazebo_ros spawn_model -file planar_RR_robot.urdf -urdf -z 1 -model planar_RR_robot
    roslaunch adaptive_controller/launch/adaptive_controller.launch

OR
    roslaunch adaptive_controller planar_RR__robot_sim.launch
*/

namespace novel_adaptive_controller_ns
{
    class NovelAdaptiveController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
        bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
        {
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

            n.getParam("gains/Kgamma", Kgamma);
            n.getParam("gains/Kr", Kr);
            n.getParam("gains/Kp", Kp);
            n.getParam("gains/Kv", Kv);

            ROS_INFO("Initialisation complete!");

            return true;
        }

        // TODO: Figure out exactly what these arguments mean
        void update(const ros::Time& time, const ros::Duration& period)
        {
            // Make the robot state variables
            Eigen::MatrixXd q_robot(2,1);
            q_robot(0,0) = joints_[0].getPosition();
            q_robot(1,0) = joints_[1].getPosition();

            Eigen::MatrixXd qd_robot(2,1);
            qd_robot(0,0) = joints_[0].getVelocity();
            qd_robot(1,0) = joints_[1].getVelocity();

            // Get the trajectory point
            Eigen::MatrixXd qt_qtd_qtdd = getTrajectory(elapsed_time);
            Eigen::MatrixXd qt_qtd_qtdd_prev = getTrajectory(elapsed_time - period.toSec());
            
            Eigen::MatrixXd qrd, qrdd, r_robot, theta_hat_d;
            if (elapsed_time > 60.0) // Stop sending commands and dump data
            {
                joints_[0].setCommand(tau_prev(0,0));
                joints_[1].setCommand(tau_prev(1,0));
                
                if (!isDataDumped)
                {
                    dumpData(sim_states_debug, "/home/kiran/dissertation/ros_experimenting_ws/src/matlab_files/data/novel_adaptive_data.csv");
                    ROS_INFO("CSV File created"); // Add the location of dump
                    isDataDumped = !isDataDumped;
                }
                return;
            }
            else if (elapsed_time > 0) // Update theta_hat
            {
                computeError(qt_qtd_qtdd_prev, q_robot, qd_robot, qrd, qrdd, r_robot);
                Eigen::MatrixXd Phi = getPhi(q_robot, qd_robot, qrd, qrdd); // Update Phi using prev trajectory and current robot states.     
                Eigen::MatrixXd identity5x5 = Eigen:: MatrixXd::Identity(5, 5);
                theta_hat_d = Kgamma*identity5x5*Phi.transpose()*r_robot;
                theta_hat = theta_hat + theta_hat_d*period.toSec(); // NOTE: May have to do interpolation if periodicity is bad.
            }

            // Compute the control torque
            computeError(qt_qtd_qtdd, q_robot, qd_robot, qrd, qrdd, r_robot);
            Eigen::MatrixXd Phi = getPhi(q_robot, qd_robot, qrd, qrdd);
            Eigen::MatrixXd tau = Phi*theta_hat + Kr*r_robot;

            joints_[0].setCommand(tau(0,0));
            joints_[1].setCommand(tau(1,0));

            std::vector<double> sim_state;
            sim_state.push_back(time.toSec());
            sim_state.push_back(period.toSec());
            sim_state.push_back(q_robot(0,0));
            sim_state.push_back(q_robot(1,0));
            sim_state.push_back(qd_robot(0,0));
            sim_state.push_back(qd_robot(1,0));
            sim_state.push_back(qt_qtd_qtdd_prev(0,0));
            sim_state.push_back(qt_qtd_qtdd_prev(1,0));
            sim_state.push_back(qt_qtd_qtdd_prev(2,0));
            sim_state.push_back(qt_qtd_qtdd_prev(3,0));
            sim_state.push_back(qt_qtd_qtdd_prev(4,0));
            sim_state.push_back(qt_qtd_qtdd_prev(5,0));
            sim_state.push_back(theta_hat(0,0));
            sim_state.push_back(theta_hat(1,0));
            sim_state.push_back(theta_hat(2,0));
            sim_state.push_back(theta_hat(3,0));
            sim_state.push_back(theta_hat(4,0));
            sim_state.push_back(tau(0,0));
            sim_state.push_back(tau(1,0));

            sim_states_debug.push_back(sim_state);

            std::cout << elapsed_time;
            std::cout << std::endl;
            // ROS_INFO("CP"); // Add the location of dump
            tau_prev = tau;
            elapsed_time += period.toSec();
            qt_qtd_qtdd_prev = qt_qtd_qtdd;
        }

        void starting(const ros::Time& time) { }
        void stopping(const ros::Time& time) { }

        Eigen::MatrixXd getTrajectory(double t)
        {
            Eigen::MatrixXd qt_qtd_qtdd(6,1);
            qt_qtd_qtdd(0,0) = deg2rad(17.4534 + sin(0.1*t + 2) + 16*sin(0.2*t + 10) + 18*sin(0.3*t + 12));
            qt_qtd_qtdd(1,0) = deg2rad(0.8189 + 8*sin(0.2*t + 2) + 6*sin(0.3*t+10) + 9*sin(0.36*t + 12));
            qt_qtd_qtdd(2,0) = deg2rad(cos(t/10 + 2)/10 + (16*cos(t/5 + 10))/5 + (27*cos((3*t)/10 + 12))/5);
            qt_qtd_qtdd(3,0) = deg2rad((8*cos(t/5 + 2))/5 + (9*cos((3*t)/10 + 10))/5 + (81*cos((9*t)/25 + 12))/25);
            qt_qtd_qtdd(4,0) = deg2rad(sin(t/10 + 2)/100 - (16*sin(t/5 + 10))/25 - (81*sin((3*t)/10 + 12))/50);
            qt_qtd_qtdd(5,0) = deg2rad((8*sin(t/5 + 2))/25 - (27*sin((3*t)/10 + 10))/50 - (729*sin((9*t)/25 + 12))/625);
            return qt_qtd_qtdd;
        }
        
        double deg2rad(double deg)
        {
            return 2*M_PI*(deg/360);
        }
        
        Eigen::MatrixXd getPhi(Eigen::MatrixXd q_robot, Eigen::MatrixXd qd_robot, Eigen::MatrixXd qrd, Eigen::MatrixXd qrdd)
        {
            const double g_const = 9.81;

            // TODO: Use the indexing instead of declaring more variables directly to save whitespace.
            double q1 = q_robot(0,0);
            double q2 = q_robot(1,0);
            double q1d = qd_robot(0,0);
            double q2d = qd_robot(1,0);
            double qr1d = qrd(0,0);
            double qr2d = qrd(1,0);
            double qr1dd = qrdd(0,0);
            double qr2dd = qrdd(1,0);

            Eigen::MatrixXd Phi(2,5);
            Phi(0,0) = qr1dd;
            Phi(0,1) = cos(q2)*(2*qr1dd + qr2dd) - sin(q2)*(q2d*qr1d + (q1d + q2d)*qr2d);
            Phi(0,2) = qr2dd;
            Phi(0,3) = g_const*cos(q1);
            Phi(0,4) = g_const*cos(q1+q2);
            Phi(1,0) = 0;
            Phi(1,1) = qr1dd*cos(q2) + q1d*qr1d*sin(q2);
            Phi(1,2) = qr1dd + qr2dd;
            Phi(1,3) = 0;
            Phi(1,4) = g_const*cos(q1 + q2);
            return Phi;
        }
        
        void computeError(Eigen::MatrixXd qt_qtd_qtdd, Eigen::MatrixXd q_robot, Eigen::MatrixXd qd_robot, Eigen::MatrixXd& qrd, Eigen::MatrixXd& qrdd, Eigen::MatrixXd& r_robot)
        {
            Eigen::MatrixXd qt(2,1);
            qt(0,0) = qt_qtd_qtdd(0,0);
            qt(1,0) = qt_qtd_qtdd(1,0);
           
            Eigen::MatrixXd qtd(2,1);
            qtd(0,0) = qt_qtd_qtdd(2,0);
            qtd(1,0) = qt_qtd_qtdd(3,0);

            Eigen::MatrixXd qtdd(2,1);
            qtdd(0,0) = qt_qtd_qtdd(4,0);
            qtdd(1,0) = qt_qtd_qtdd(5,0);

            Eigen::MatrixXd e_robot = qt - q_robot;
            Eigen::MatrixXd ed_robot = qtd - qd_robot;

            qrd = qtd + Kp*e_robot;
            qrdd = qtdd + Kv*ed_robot;
            r_robot = qrd - qd_robot;

            return;
        }
        
        void dumpData(std::vector<std::vector<double>> data, std::string fname)
        {
            std::ofstream myfile;
            myfile.open (fname);

            for (size_t i = 0; i < data.size(); i++)
            {
                for (size_t j = 0; j < data[i].size(); j++)
                {
                    myfile << data[i][j];
                    if (j < data[i].size() - 1)
                    {
                        myfile << ", ";
                    }
                }
                myfile << "\r\n";
            }
            myfile.close();
        }

        public:
            NovelAdaptiveController() : theta_hat(5,1){ 
                // Initialise theta_hat
                theta_hat(0,0) = m1*pow(l1,2) + m2*pow(l1,2) + m2*pow(l2,2) + Izz1 + Izz2;
                theta_hat(1,0) = m2*l1*l2;
                theta_hat(2,0) = m2*pow(l2,2) + Izz2;
                theta_hat(3,0) = m1*l1 + m2*l1;
                theta_hat(4,0) = m2*l2;
            }
        
        private:
            hardware_interface::JointHandle joints_[2];
            double command_[2];
            ros::Subscriber sub_command_;
            Eigen::MatrixXd theta_hat;
            Eigen::MatrixXd tau_prev;
            double Kr, Kv, Kp, Kgamma;
            double elapsed_time = 0;

            // Debug variables
            std::vector<std::vector<double>> sim_states_debug;
            bool isDataDumped = false;

    };

    PLUGINLIB_EXPORT_CLASS(novel_adaptive_controller_ns::NovelAdaptiveController, controller_interface::ControllerBase);
}

