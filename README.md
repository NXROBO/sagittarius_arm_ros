# sagittarius_arm_ros
下载安装
mkdir -p ~/sagittarius_ws/src
cd ~/sagittarius_ws/src
git clone https://github.com/nxrobo/sagittarius_arm_ros.git
cd sagittarius_arm_ros
./install.sh
cd ~/sagittarius_ws
catkin_make
source devel/setup.bash

加载工作空间
source ~/sagittarius_ws/devel/setup.bash

单个舵机控制
roslaunch sdk_sagittarius_arm rviz_control_sagittarius.launch

ros moveit界面的操作:
roslaunch sagittarius_moveit demo_true.launch

录制动作
source ~/sagittarius_ws/devel/setup.bash
roslaunch sagittarius_puppet_control puppet_control_single.launch bag_name:="record1"

播放重复动作
source ~/sagittarius_ws/devel/setup.bash
roslaunch sdk_sagittarius_arm run_sagittarius.launch
cd ~/sagittarius_ws/src/sagittarius_arm_ros/sagittarius_demo/sagittarius_puppet_control/bag
rosbag play record1.bag

精度测试：
source ~/sagittarius_ws/devel/setup.bash
roslaunch sagittarius_moveit demo_true.launch
rosrun sdk_sagittarius_arm precision_test.py
