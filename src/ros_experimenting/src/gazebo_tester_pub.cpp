// Gazebo dependencies
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>

// ROS dependencies
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <iostream>

// This PID controller is hardcoded to be used for the katana_motor2_lift_joint from the katana robot model

ros::Publisher pub;
double set_point = 0.10;
double k_p = 10.0;
const double applied_torque = 5000;

// Function is called everytime a message is received.
void cb(const sensor_msgs::JointState::ConstPtr &_msg) // TODO: Figure out why some IDs are not being populated. (prob a Gazebo issue)
{
  const sensor_msgs::JointState* joint = _msg.get();
  int length = joint->name.size();
  std::vector<double> pos = _msg->position;
  std::vector<double> vel = _msg->velocity;

  // DEBUG
  for (int i=0; i<length; ++i)
  {
    std::string joint_name = joint->name[i];
    std::string position_name = std::to_string(joint->position[i]);
    std::string velocity_name = std::to_string(joint->velocity[i]);
    // std::string effort_name = std::to_string(joint->effort[i]);

    // ROS_INFO("Joint Name: %s", joint_name.c_str());    
    ROS_INFO("Joint Position: %s", position_name.c_str());    
    ROS_INFO("Joint Velocity: %s", velocity_name.c_str());    
    // ROS_INFO("Joint Effort: %s", effort_name.c_str());
  }
 
  // Compute the required control input
  double curr_pos = joint->position[0];
  double error = set_point - curr_pos;

  // // Ensure the directionalities are being considered
  // int signum_curr_pos = fabs(curr_pos)/curr_pos;
  // int signum_set_point = fabs(set_point)/set_point;
  // if (signum_curr_pos != signum_set_point)
  //   error = set_point + curr_pos;
  double control_input = k_p * error;

  // ROS_INFO("signum_curr_pos: %d", signum_curr_pos);  
  // ROS_INFO("signum_set_point: %d", signum_set_point);  
  ROS_INFO("Control Input: %f", control_input);   

  geometry_msgs::Vector3 msgWrenchForce;
  msgWrenchForce.x = 0;
  msgWrenchForce.y = 0;
  msgWrenchForce.z = 0;

  geometry_msgs::Vector3 msgWrenchTorque;
  msgWrenchTorque.x = 0;
  msgWrenchTorque.y = 0;
  msgWrenchTorque.z = control_input;

  geometry_msgs::Wrench msgWrench;
  // msgWrench.header.stamp = ros::Time::now();
  msgWrench.force = msgWrenchForce;
  msgWrench.torque = msgWrenchTorque;
  pub.publish(msgWrench);
}

int main(int argc, char **argv)
{
  // ROS
  ros::init(argc, argv, "publisher");
  ros::NodeHandle n;

  // Start the publisher
  ROS_INFO("Starting Publisher");
  std::string ros_topic_to_pub = "/force/katana_motor2_lift_link";
  pub = n.advertise<geometry_msgs::Wrench>(ros_topic_to_pub, 100);

  // Subscribe to the joint states topic
  ros::Subscriber sub = n.subscribe("/joint_states", 1, cb); // 1 because we want to use most recent sensor reading.

  ros::Rate loop_rate(1000);
  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}