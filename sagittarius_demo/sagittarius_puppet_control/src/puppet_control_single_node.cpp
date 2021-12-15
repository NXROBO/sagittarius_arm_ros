#include <ros/ros.h>
#include "sdk_sagittarius_arm/ArmRadControl.h"
#include "sdk_sagittarius_arm/ArmInfo.h"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <string>
sensor_msgs::JointState joint_states;         // globally available joint_states message

/// @brief Joint state callback function
/// @param msg - updated joint states
void joint_state_cb(const sensor_msgs::JointState &msg)
{
  joint_states = msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sagittarius_puppet_control_single");
  ros::NodeHandle n;

  // Subscribe to the robot's joint states and publish those states as joint commands so that rosbag can record them
  ros::Subscriber sub_positions = n.subscribe("joint_states", 100, joint_state_cb);
  ros::Publisher pub_positions = n.advertise<sdk_sagittarius_arm::ArmRadControl>("joint/commands", 100);
  ros::Publisher pub_gripper = n.advertise<std_msgs::Float64>("gripper/command", 100);
  // Get some robot info to figure out how many joints the robot has
  ros::ServiceClient srv_robot_info = n.serviceClient<sdk_sagittarius_arm::ArmInfo>("get_robot_info");
  ros::Publisher pub_torque = n.advertise<std_msgs::String>("/control_torque", 1);
  ros::Rate loop_rate(100);
  bool success;

  // Wait for the 'arm_node' to finish initializing
  while ((pub_positions.getNumSubscribers() < 1 || joint_states.position.size() < 1) && ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  // Torque off the robot arm so that the user can easily manipulate it. Conveniently, this also prevents
  // the motors from acting upon the position commands being sent to the 'joint/commands' topic
  std_msgs::String msg;
  msg.data = "close";
  pub_torque.publish(msg);

  // Get the number of joints that the first robot has
  sdk_sagittarius_arm::ArmInfo robot_info_srv;
  success = srv_robot_info.call(robot_info_srv);
  if (!success)
  {
    ROS_ERROR("Could not get robot info.");
    return 1;
  }

  size_t cntr = 0;
  while (ros::ok())
  {
    // put joint positions from the robot as position commands for itself
    sdk_sagittarius_arm::ArmRadControl pos_msg;
    for (size_t i{0}; i < robot_info_srv.response.num_joints; i++)
    {
      pos_msg.rad.push_back(joint_states.position.at(i));
    }
    pub_positions.publish(pos_msg);

    // Every 10 iterations, send a gripper position command. If this is sent every loop iteration,
    // it will cause the driver node to lag (since the port needs to be accessed twice per loop iteration instead of just once)
    if (cntr == 10)
    {
      std_msgs::Float64 gpr_msg;
      gpr_msg.data = joint_states.position.at(joint_states.position.size()-1);
      pub_gripper.publish(gpr_msg);
            ROS_WARN("NUM IS %d, at %d,%lf",robot_info_srv.response.num_joints,joint_states.position.size()-1, gpr_msg.data);

      cntr = 0;
    }
    cntr++;
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
