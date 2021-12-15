#include <ros/ros.h>
#include <iostream>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_msgs/Float32MultiArray.h>
 
#include <sstream>
#include <stdlib.h>
#include <sys/types.h>
#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <signal.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
 
using namespace std;
 
typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;
 
std::basic_string<char> Link1;
std::basic_string<char> Link2;
std::basic_string<char> Link3;
std::basic_string<char> Link4;
std::basic_string<char> Link5;
std::basic_string<char> Link6;

/* 目标位置 */
double cp1;
double cp2;
double cp3;
double cp4;
double cp5;
double cp6;
 
/* 目标位置 */
double tp1;
double tp2;
double tp3;
double tp4;
double tp5;
double tp6;
     actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> *joint_action_server;    // Action Server that responds to MoveIt's Action Client to control the joints (excludes gripper)
/* 收到action的goal后调用的回调函数 */
void execute(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal, Server* as)
{
	//ros::Rate rate(1);
 
    Link1 = goal->trajectory.joint_names[0];
    Link2 = goal->trajectory.joint_names[1];
    Link3 = goal->trajectory.joint_names[2];
    Link4 = goal->trajectory.joint_names[3];
    Link5 = goal->trajectory.joint_names[4];
    Link6 = goal->trajectory.joint_names[5];
 
    cp1 = goal->trajectory.points[0].positions[0];
    cp2 = goal->trajectory.points[0].positions[1];
    cp3 = goal->trajectory.points[0].positions[2];
    cp4 = goal->trajectory.points[0].positions[3];
    cp5 = goal->trajectory.points[0].positions[4];
    cp6 = goal->trajectory.points[0].positions[5];
 
    tp1 = goal->trajectory.points[1].positions[0];
    tp2 = goal->trajectory.points[1].positions[1];
    tp3 = goal->trajectory.points[1].positions[2];
    tp4 = goal->trajectory.points[1].positions[3];
    tp5 = goal->trajectory.points[1].positions[4];
    tp6 = goal->trajectory.points[1].positions[5];
    printf("fffffffff\n");
	// 并且按照1hz的频率发布进度feedback
	// control_msgs::FollowJointTrajectoryFeedback feedback;
    //feedback = NULL;
    //as->publishFeedback(feedback);
 
    // 当action完成后，向客户端返回结果
    //printf("goal=[%f,%f,%f,%f,%f,%f]\n",tp1,tp2,tp3,tp4,tp5,tp6);
    ROS_INFO("Recieve action successful!");
    as->setSucceeded();
}
 
void arm_joint_trajectory_action_callback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
{
  control_msgs::FollowJointTrajectoryResult result;
printf("ffffffsssssss\n");
  ROS_WARN("RobotArm::arm_joint_trajectory_action_callback");

} 
/* 主函数主要用于动作订阅和套接字通信 */
int main(int argc, char** argv)
{
	ros::init(argc, argv, "trajectory_server");
	ros::NodeHandle nh;
	// 定义一个服务器
	//Server server(nh, "/sagittarius_arm_controller/follow_joint_trajectory", boost::bind(&execute, _1, &server), false);
	// 服务器开始运行
	//server.start();



    joint_action_server = new actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>
                              (nh, "sagittarius_arm_controller/follow_joint_trajectory", boost::bind(&arm_joint_trajectory_action_callback,  _1),false);
    joint_action_server->start();
	ros::spin();
    delete     joint_action_server;
 
}
 

