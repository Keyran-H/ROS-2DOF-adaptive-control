#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>

// TODO: Programmaticaly get the joint names from the parameter server
// NOTE: I can use a CSV Parser to populate the points if I want to get fancy

int main(int argc, char** argv)
{
    // Init the ROS node
    ros::init(argc, argv, "traj_pub");
    ros::NodeHandle n;
    ros::Publisher pub;
    pub = n.advertise<trajectory_msgs::JointTrajectory>("command", 1);

    trajectory_msgs::JointTrajectory robotTrajMsg;

    
    // Joints
    robotTrajMsg.joint_names.push_back("planar_RR_joint1");
    robotTrajMsg.joint_names.push_back("planar_RR_joint2");
   
    robotTrajMsg.points.resize(2);
        
    ///// First Point
    int idx = 0;

    // Joint Position
    robotTrajMsg.points[idx].positions.resize(2);
    robotTrajMsg.points[idx].positions[0] = 0.0;
    robotTrajMsg.points[idx].positions[1] = 0.0;
    
    // Joint Velocity
    robotTrajMsg.points[idx].velocities.resize(2);
    for (size_t i = 0; i < 2; i++)
    {
        robotTrajMsg.points[idx].velocities[i] = 0.0;
    }    

    // Joint Accelerations
    robotTrajMsg.points[idx].accelerations.resize(2);
    for (size_t i = 0; i < 2; i++)
    {
        robotTrajMsg.points[idx].accelerations[i] = 0.0;
    }

    // Desired time
    robotTrajMsg.points[idx].time_from_start = ros::Duration(1.0);

    ///// Second Point
    idx++;
    
    // Joint Position
    robotTrajMsg.points[idx].positions.resize(2);
    robotTrajMsg.points[idx].positions[0] = 1.53;
    robotTrajMsg.points[idx].positions[1] = 1.62;
    
    // Joint Velocity
    robotTrajMsg.points[idx].velocities.resize(2);
    for (size_t i = 0; i < 2; i++)
    {
        robotTrajMsg.points[idx].velocities[i] = 0.0;
    }    

    // Joint Accelerations
    robotTrajMsg.points[idx].accelerations.resize(2);
    for (size_t i = 0; i < 2; i++)
    {
        robotTrajMsg.points[idx].accelerations[i] = 0.0;
    }

    // Desired time
    robotTrajMsg.points[idx].time_from_start = ros::Duration(2.0);

    // Publish the Trajectory
    pub.publish(robotTrajMsg);

    // Consider removing this
    ros::Rate loop_rate(5000);
    while(ros::ok())
    {
    ros::spinOnce();
    loop_rate.sleep();
    }

  return 0;
}