// Gazebo dependencies
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>

// ROS dependencies
#include <geometry_msgs/WrenchStamped.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <iostream>

ros::Publisher pub;

int main(int argc, char **argv)
{
  // ROS
  ros::init(argc, argv, "publisher");
  ros::NodeHandle n;

  // Load gazebo
  gazebo::client::setup(argc, argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node()); // TODO: What exactly does this line do?
  node->Init();

  // std::string ros_topic_to_pub = "/gazebo/default/model_1/link_1/wrench";
  std::string ros_topic_to_pub = "~/user_cmd";
  ROS_INFO("Starting Publisher");
  pub = n.advertise<geometry_msgs::WrenchStamped>(ros_topic_to_pub, 100);

  // Populate the WrenchStamped message
  geometry_msgs::WrenchStamped msgWrenchedStamped;
  msgWrenchedStamped.header.stamp = ros::Time::now();
  msgWrenchedStamped.wrench.force.x = 0;
  msgWrenchedStamped.wrench.force.y = 500;
  msgWrenchedStamped.wrench.force.z = 0;
  msgWrenchedStamped.wrench.torque.x = 0;
  msgWrenchedStamped.wrench.torque.y = 0;
  msgWrenchedStamped.wrench.torque.z = 0;

  ROS_INFO("Publishing...");
  pub.publish(msgWrenchedStamped);

  gazebo::client::shutdown();

  return 0;
}