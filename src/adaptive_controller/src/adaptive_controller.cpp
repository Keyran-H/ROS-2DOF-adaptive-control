#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>
#include <fstream>
#include <ros/package.h>
#include <Eigen/Core>

// NOTE: This adaptive controller is hardcoded to work at 1kHz. Frequency is set in sim.world and trajectory is set using trajectory_1000Hz.csv file.

// Initial Estimates. 
// TODO: Move initial estimates to .yaml file
// NOTE: There is no mismatch between real and sim parameters
#define m1 1
#define m2 1
#define l1 1
#define l2 1
#define Izz1 0.00166666666667
#define Izz2 0.00166666666667

namespace adaptive_controller_ns
{
    class AdaptiveController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
        bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
        {
            // Get relative package path
            pckg_path = ros::package::getPath("adaptive_controller");

            // Read from the csv file
            std::ifstream file(pckg_path + "/trajectory/trajectory_1000Hz.csv");
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

            n.getParam("gains/Kgamma", Kgamma);
            n.getParam("gains/Kr", Kr);
            n.getParam("gains/Kp", Kp);
            n.getParam("gains/Kv", Kv);

            ROS_INFO("Initialisation complete!");

            return true;
        }

        void update(const ros::Time& time, const ros::Duration& period)
        {
            // Read the robot state variables
            Eigen::MatrixXd q_robot(2,1);
            q_robot(0,0) = joints_[0].getPosition();
            q_robot(1,0) = joints_[1].getPosition();

            Eigen::MatrixXd qd_robot(2,1);
            qd_robot(0,0) = joints_[0].getVelocity();
            qd_robot(1,0) = joints_[1].getVelocity();

            Eigen::MatrixXd qrd, qrdd, r_robot, theta_hat_d;
                        
            if (traj_idx > trajectory.size() - 1) // At last traj point, dump data and set joint state refs to last traj point.
            {   
                traj_idx = trajectory.size() - 1;     
                if (!isDataDumped)
                {
                    std::string dumpfile = pckg_path + "/datadump/trajectory_data_1000Hz.csv";
                    dumpData(sim_states_debug, dumpfile);
                    ROS_INFO("CSV File created: %s", dumpfile.c_str());
                    isDataDumped = !isDataDumped;
                }
            }
            if (traj_idx > 0) // Update theta_hat
            { 
                computeError(traj_idx - 1, q_robot, qd_robot, qrd, qrdd, r_robot);
                Eigen::MatrixXd Phi = getPhi(q_robot, qd_robot, qrd, qrdd); // Update Phi using prev trajectory and current robot states.     
                Eigen::MatrixXd identity5x5 = Eigen:: MatrixXd::Identity(5, 5);
                theta_hat_d = Kgamma*identity5x5*Phi.transpose()*r_robot;
                theta_hat = theta_hat + theta_hat_d*period.toSec(); // NOTE: May have to do interpolation if periodicity is bad.
            }

            // Compute the control torque
            computeError(traj_idx, q_robot, qd_robot, qrd, qrdd, r_robot);
            Eigen::MatrixXd Phi = getPhi(q_robot, qd_robot, qrd, qrdd);
            Eigen::MatrixXd tau = Phi*theta_hat + Kr*r_robot;

            joints_[0].setCommand(tau(0,0));
            joints_[1].setCommand(tau(1,0));

            if (!isDataDumped)
            {
                std::vector<double> sim_state;
                sim_state.push_back(time.toSec());
                sim_state.push_back(period.toSec());
                sim_state.push_back(q_robot(0,0));
                sim_state.push_back(q_robot(1,0));
                sim_state.push_back(qd_robot(0,0));
                sim_state.push_back(qd_robot(1,0));
                sim_state.push_back(trajectory[traj_idx][0]); // qt(0,0)
                sim_state.push_back(trajectory[traj_idx][1]); // qt(1,0)
                sim_state.push_back(trajectory[traj_idx][2]); // qtd(0,0)
                sim_state.push_back(trajectory[traj_idx][3]); // qtd(1,0)
                sim_state.push_back(trajectory[traj_idx][4]); // qtdd(0,0)
                sim_state.push_back(trajectory[traj_idx][5]); // qtdd(1,0)
                sim_state.push_back(theta_hat(0,0));
                sim_state.push_back(theta_hat(1,0));
                sim_state.push_back(theta_hat(2,0));
                sim_state.push_back(theta_hat(3,0));
                sim_state.push_back(theta_hat(4,0));
                sim_state.push_back(tau(0,0));
                sim_state.push_back(tau(1,0));
                sim_states_debug.push_back(sim_state);
            }

            traj_idx++;
        }

        void starting(const ros::Time& time) { }
        void stopping(const ros::Time& time) { }

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
        
        void computeError(unsigned int idx, Eigen::MatrixXd q_robot, Eigen::MatrixXd qd_robot, Eigen::MatrixXd& qrd, Eigen::MatrixXd& qrdd, Eigen::MatrixXd& r_robot)
        {
            Eigen::MatrixXd qt(2,1);
            qt(0,0) = trajectory[idx][0];
            qt(1,0) = trajectory[idx][1];
           
            Eigen::MatrixXd qtd(2,1);
            qtd(0,0) = trajectory[idx][2];
            qtd(1,0) = trajectory[idx][3];

            Eigen::MatrixXd qtdd(2,1);
            qtdd(0,0) = trajectory[idx][4];
            qtdd(1,0) = trajectory[idx][5];

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
            std::vector<std::vector<double>> trajectory;
            Eigen::MatrixXd theta_hat;
            unsigned int traj_idx = 0;
            double Kr, Kv, Kp, Kgamma;
            std::string pckg_path;

            // Debug variables
            std::vector<std::vector<double>> sim_states_debug;
            bool isDataDumped = false;

    };

    PLUGINLIB_EXPORT_CLASS(adaptive_controller_ns::AdaptiveController, controller_interface::ControllerBase);
}

