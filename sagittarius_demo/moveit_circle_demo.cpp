#include<math.h>
#include<ros/ros.h>
#include<moveit/move_group_interface/move_group_interface.h>
#include<moveit/robot_trajectory/robot_trajectory.h>

int main(int argc, char **argv)
{
    ros::init(argc,argv,"moveit_circle_demo");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("xmate_arm");
    std::string end_effector_link = arm.getEndEffectorLink();
    std::string reference_frame = "base_link";
    arm.setPoseReferenceFrame(reference_frame);

    //当运动规划失败后，允许重新规划
    arm.allowReplanning(true);

    //设置位置(单位：米)和姿态（单位：弧度）的允许误差
    arm.setGoalPositionTolerance(0.001);
    arm.setGoalOrientationTolerance(0.01);

    //设置允许的最大速度和加速度
    arm.setMaxAccelerationScalingFactor(0.8);
    arm.setMaxVelocityScalingFactor(0.8);

    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);

    // 设置机器人终端的目标位置
    geometry_msgs::Pose target_pose;//设定圆心的位置
    target_pose.orientation.x = 0.14858;
    target_pose.orientation.y = 0.73236;
    target_pose.orientation.z = -0.03444;
    target_pose.orientation.w = 0.66362;

    target_pose.position.x = -0.050308;
    target_pose.position.y = -0.039851;
    target_pose.position.z = 0.78264;

    arm.setPoseTarget(target_pose);
    arm.move();//首先运动到圆心的位置
    sleep(1);


	std::vector<geometry_msgs::Pose> waypoints;

    //将初始位姿加入路点列表
	waypoints.push_back(target_pose);

    double centerA = target_pose.position.x;
    double centerB = target_pose.position.z;
    double radius = 0.13;

    for(double th=0.0; th<6.28; th=th+0.01)
    {
        target_pose.position.y = centerA + radius * cos(th);
        target_pose.position.z = centerB + radius * sin(th);
        waypoints.push_back(target_pose);
    }

	// 笛卡尔空间下的路径规划
	moveit_msgs::RobotTrajectory trajectory;
	const double jump_threshold = 0.0;
	const double eef_step = 0.01;
	double fraction = 0.0;
    int maxtries = 100;   //最大尝试规划次数
    int attempts = 0;     //已经尝试规划次数

    while(fraction < 1.0 && attempts < maxtries)
    {
        fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        attempts++;
        
        if(attempts % 10 == 0)
            ROS_INFO("Still trying after %d attempts...", attempts);
    }
    
    if(fraction == 1)
    {   
        ROS_INFO("Path computed successfully. Moving the arm.");

	    // 生成机械臂的运动规划数据
	    moveit::planning_interface::MoveGroupInterface::Plan plan;
	    plan.trajectory_ = trajectory;

	    // 执行运动
	    arm.execute(plan);
        sleep(1);
    }
    else
    {
        ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
    }

    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);

	ros::shutdown(); 
	return 0;
}    

