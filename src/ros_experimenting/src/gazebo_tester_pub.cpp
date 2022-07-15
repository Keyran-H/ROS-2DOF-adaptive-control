// Gazebo dependencies
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>

// ROS dependencies
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <iostream>

ros::Publisher pub;

// Function is called everytime a message is received.
void cb(const sensor_msgs::JointState::ConstPtr &_msg) // TODO: Figure out why some IDs are not being populated. (prob a Gazebo issue)
{
  const sensor_msgs::JointState* joint = _msg.get();
  int length = joint->name.size();
  std::vector<double> pos =  _msg->position;
  std::vector<double> vel = _msg->velocity;

  // Included this for my sanity purposes
  for (int i=0; i<length; ++i)
  {
    std::string joint_name = joint->name[i];
    ROS_INFO("Joint Name: %s", joint_name.c_str());
    std::string position_name = std::to_string(joint->position[i]);
    ROS_INFO("Joint Position: %s", position_name.c_str());
    std::string velocity_name = std::to_string(joint->velocity[i]);
    ROS_INFO("Joint Velocity: %s", velocity_name.c_str());
    std::string effort_name = std::to_string(joint->velocity[i]);
    ROS_INFO("Joint Velocity: %s", effort_name.c_str());
  }
 
  // std::cout << _msg->effort;

  // Dump the message contents to stdout.
  // std::cout << _msg;
  // geometry_msgs::WrenchStamped msgStamped;
  // // try WrenchStamped msgStamped;
  // msgWrenched.header.stamp = ros::Time::now();
  // msgWrenched.wrench.force.x = 0;
  // msgWrenched.wrench.force.y = 0;
  // msgWrenched.wrench.force.z = 0;
  // msgWrenched.wrench.torque.x = 0;
  // msgWrenched.wrench.torque.y = 0;
  // msgWrenched.wrench.torque.z = 500;
  // pub.publish(msgWrenchedStamped);
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

  ros::Rate loop_rate(10);
  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}