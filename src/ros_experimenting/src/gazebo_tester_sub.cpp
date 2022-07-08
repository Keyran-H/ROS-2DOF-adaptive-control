#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include "ros/ros.h"
#include "std_msgs/String.h"

// TODO: Figure out how can I know which are the correct header files.

/*

How to run this?
1. roscore
2. Start the gazebo sim first: gazebo random_files/force_torque_tutorial.world
3. Run the listner: rosrun ros_experimenting listener


*/

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

// Function is called everytime a message is received.
// void cb(ConstWorldStatisticsPtr &_msg)
void cb(ConstForceTorquePtr &_msg) // TODO: Figure out why some IDs are not being populated. (prob a Gazebo issue)
{
  // Dump the message contents to stdout.
  std::cout << _msg->DebugString();
}

int main(int argc, char **argv)
{
    // Load Gazebo & ROS
  gazebo::client::setup(argc, argv);

  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  // Load gazebo
  gazebo::client::setup(argc, argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Listen to Gazebo world_stats topic
//   gazebo::transport::SubscriberPtr gazebo_sub = node->Subscribe("~/world_stats", cb);
gazebo::transport::SubscriberPtr gazebo_sub = node->Subscribe("/gazebo/default/model_1/joint_01/force_torque/wrench", cb);

  // Busy wait loop...replace with your own code as needed.
  while (true)
    gazebo::common::Time::MSleep(10);

  // Make sure to shut everything down.
  gazebo::client::shutdown();

  ros::spin();

  return 0;
}