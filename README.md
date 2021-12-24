# NXROBO Spark sagittarius_arm_ros



## 说明 Description
- 本说明为初学者体验版，一个基于六个自由度和末端夹具的机械臂。可以用来学习moveit的设备。
## 列表 Table of Contents

* [功能包说明packages-overview](#功能包说明packages-overview)
* [使用usage](#使用usage)
* [视频展示Video](#视频展示Video)

## 功能包说明packages-overview

* ***sagittarius_demo*** : 机械臂的DEMO。
* ***sagittarius_descriptions*** : 机械臂的描述功能包。
* ***sagittarius_moveit*** : 机械臂的moveit功能包。
* ***sagittarius_toolboxes*** : 机械臂的基础工具箱。
* ***sak_sagittarius_arm*** : 机械臂的SDK源码。
* ***install.sh*** : 安装脚本。
## 使用usage

### 系统要求 Prequirement

* System:	Ubuntu 16.04 ,Ubuntu 18.04 or Ubuntu 20.04
* ROS Version:	kinetic, melodic or noetic

### 下载安装 Download and install
* 创建工作空间 Creat the workspace:
```yaml
    mkdir -p ~/sagittarius_ws/src
    cd ~/sagittarius_ws/src
```
* 下载源代码 Download the workspace:
```yaml
    git clone https://github.com/NXROBO/sagittarius_arm_ros.git
```
* 安装依赖库 Install libraries and dependencies:
```yaml
    cd sagittarius_arm_ros
    ./install.sh
```
### 编译运行 compile and run
```yaml
    cd ~/sagittarius_ws
    catkin_make
```
* 如果编译一切正常，可根据提示运行相关例程。If everything goes fine, test the examples as follow:
```yaml
    source devel/setup.bash
    roslaunch roslaunch sagittarius_moveit demo_true.launch
```

## 视频展示Video
