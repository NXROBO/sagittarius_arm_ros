/* 
 * Software License Agreement (BSD License)
 * Copyright (c) 2021, NXROBO.
 * All rights reserved.
 * Author: litian.zhuang  <litian.zhuang@nxrobo.com>   
 */
#include <sdk_sagittarius_arm/sdk_sagittarius_arm_common.h>

#include <cstdio>
#include <cstring>
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h> 
namespace sdk_sagittarius_arm
{

    CSDarmCommon::CSDarmCommon(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
        mDiagPublisher(NULL),
        dExpectedFreq(15.0), /* Default frequency */
        mThrcv(NULL),
        mNodeHandler(nh),
        mPrivNodeHandler(pnh)
    {
        /*Initialize receive buffer*/
        memset(mRecvBuffer, RECV_BUFFER_SIZE, 0);
        mDataLength = 0;

        /*Set reconfigure callback*/
        dynamic_reconfigure::Server<sdk_sagittarius_arm::SDKSagittariusArmConfig>::CallbackType f;
        f = boost::bind(&sdk_sagittarius_arm::CSDarmCommon::UpdateConfig, this, _1, _2);
        mDynaReconfigServer.setCallback(f);

        /*Set data publisher (used for debug)*/
        mPrivNodeHandler.param<bool>("publish_datagram", mPublishData, false);
        if(mPublishData)
        {
            /*datagram publish is enabled*/
            mDataPublisher = mNodeHandler.advertise<std_msgs::String>("datagram", 1000);
        }

        /*Set scan publisher*/
        mScanPublisher = mNodeHandler.advertise<sensor_msgs::LaserScan>("scan", 1000);

        mDiagUpdater.setHardwareID("none");
        mDiagPublisher = new diagnostic_updater::DiagnosedPublisher<sensor_msgs::LaserScan>(mScanPublisher,
                                                                                            mDiagUpdater,
                                                                                            /* frequency should be target +- 10% */
                                                                                            diagnostic_updater::FrequencyStatusParam(&dExpectedFreq, &dExpectedFreq, 0.1, 10),
                                                                                            /*timestamp delta can be from 1.1 to 1.3x what it ideally is*/
                                                                                            diagnostic_updater::TimeStampStatusParam(-1, 1.3 * 1.0 / dExpectedFreq - mConfig.time_offset));
        motionPlan_pub = mNodeHandler.advertise<sensor_msgs::JointState>("/joint_states",10);
        ROS_ASSERT(mDiagPublisher);
    }

    /**
             * 	????????????????
             */
    bool CSDarmCommon::DestroyThread(boost::thread **th)
    {

        if((*th) != NULL)
        {
            (*th)->interrupt();
            (*th)->join();
            delete (*th);
            (*th) = NULL;
            return true;
        }
        return true;
    }


    int CSDarmCommon::StopArmLock()
    {
        int result = 0;

        return result;
    }

    bool CSDarmCommon::RebootDevice()
    {
        return true;
    }

    CSDarmCommon::~CSDarmCommon()
    {
        DestroyThread(&mThrcv);
        delete mDiagPublisher;
        printf("sdk_sagittarius_arm drvier exiting.\n");
    }

    int CSDarmCommon::Init()
    {
        int result = InitDevice();
        if(0 != result)
        {
            ROS_FATAL("Failed to init device: %d", result);
            return result;
        }

        result = InitArm();
        if(0 != result)
        {
            ROS_FATAL("Failed to init scanner: %d", result);
        }

        return result;
    }

    unsigned char CSDarmCommon::CheckSum(unsigned char *buf)
    {
        int i;
        unsigned char sum = 0;
        for(i=0; i<buf[2]; i++)
        {
            sum += buf[3+i];
        }
        //printf("sum is %02x\n",sum);
        return sum;
    }

    int CSDarmCommon::InitArm()
    {
        SendArmLockOrFree(1);
        return ExitSuccess;
    }

    void CSDarmCommon::print_hex(unsigned char *buf, int len)
    {
    #if DEBUG
        int i;
        for(i=0; i<len; i++)
        {
            printf("%02x ",buf[i]);
        }
        printf("\n");
    #endif
    }

    void CSDarmCommon::StartReceiveSerail()
    {
        if(mThrcv == NULL)
            mThrcv = new boost::thread(boost::bind(&CSDarmCommon::LoopRcv, this));
    }
    void CSDarmCommon::LoopRcv()
    {
        while(ros::ok())
        {
            LoopOnce();
        }

    }

    /// @brief  ROS Publishers JointState
    void CSDarmCommon::PublishJointStates(unsigned char *buf)
    {
        sensor_msgs::JointState 		joint_state;
        struct  timeval    tv;
        struct  timezone   tz;
        gettimeofday(&tv,&tz);
        static long long ct,lt;



        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(8);
        joint_state.position.resize(8);
        joint_state.name[0] = "joint1";
        joint_state.position[0] = (short(buf[0]|buf[1]<<8))/1800.0*PI;
        joint_state.name[1] = "joint2";
        joint_state.position[1] = (short(buf[2]|buf[3]<<8))/1800.0*PI;
        joint_state.name[2] = "joint3";
        joint_state.position[2] = (short(buf[4]|buf[5]<<8))/1800.0*PI;         //?????????????????? 1
        joint_state.name[3] = "joint4";
        joint_state.position[3] = (short(buf[6]|buf[7]<<8))/1800.0*PI;
        joint_state.name[4] = "joint5";
        joint_state.position[4] = (short(buf[8]|buf[9]<<8))/1800.0*PI;
        joint_state.name[5] = "joint6";
        joint_state.position[5] = (short(buf[10]|buf[11]<<8))/1800.0*PI;
        joint_state.name[6] = "joint_gripper_left";
        joint_state.position[6] = -(short(buf[12]|buf[13]<<8))*0.026/900.0;
        joint_state.name[7] = "joint_gripper_right";
        joint_state.position[7] = -(short(buf[12]|buf[13]<<8))*0.026/900.0;
        //	printf("[[[[%f,%f]]]]\n",joint_state.position[6],joint_state.position[7]);
        /*	joint_state.name[6] = "nx07";
                    joint_state.position[6] = 0;*/
        ct = tv.tv_sec*800000+tv.tv_usec;
        if(1)//ct>=lt)
        {
            //ROS_WARN("ARM->ROS:[%f,%f,%f,%f,%f,%f]", joint_state.position[0],joint_state.position[1],joint_state.position[2],joint_state.position[3],joint_state.position[4],joint_state.position[5]);
            joint_states = joint_state;
            motionPlan_pub.publish(joint_state);
            lt = ct+100000;
        }
    }




    int CSDarmCommon::LoopOnce()
    {
        mDiagUpdater.update();
        int dataLength = 0;
        int result = GetDataGram(mRecvBuffer, RECV_BUFFER_SIZE, &dataLength);
        if(result != ExitSuccess)
        {
            ROS_ERROR("sdk_sagittarius_arm - Read Error when getting datagram: %d", result);
            mDiagUpdater.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR,
                                   "sdk_sagittarius_arm - Read Error when getting datagram.");
            return ExitError;
        }
        else
        {
            if(dataLength>0)
            {
                if((dataLength<255)&&(mDataLength<255))
                {
                    print_hex(mRecvBuffer, dataLength);
                    memcpy(mFrameBuffer+mDataLength, mRecvBuffer, dataLength);
                    mDataLength = mDataLength+dataLength;
                    if(mFrameBuffer[0] != 0x55)
                        mDataLength = 0;
                }
                else
                    mDataLength = 0;
                if((mDataLength>3)&&(mDataLength >= (mFrameBuffer[2]+5))) //??????????????????
                {
                    if((mFrameBuffer[0]==0x55)&&(mFrameBuffer[1]==0xAA)&&(mFrameBuffer[mFrameBuffer[2]+4]==0x7D)&&(CheckSum(mFrameBuffer)==(unsigned char)mFrameBuffer[mFrameBuffer[2]+3]))  //????????????????????????
                    {
                        //printf("receive data:");
                        print_hex(mFrameBuffer, mDataLength);
                        if(mFrameBuffer[4]==0x0A)  //?????????????????????
                        {
                            printf("???????????????\n");
                        }
                        else if(mFrameBuffer[4]==0x09)  //version?????????
                        {
                            if(mFrameBuffer[3]==0x02)
                                printf("version is %s\n",mFrameBuffer+5);
                        }
                        else if(mFrameBuffer[4]==0x06)  //version?????????
                        {
                            if(mFrameBuffer[3]==0x01)
                            {
                                PublishJointStates(mFrameBuffer+5);
                                //ROS_INFO("ARM->ROS:length=%d [%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f]", mDataLength, (float(short(mFrameBuffer[5]|(mFrameBuffer[6]<<8))))/10,(float(short(mFrameBuffer[7]|(mFrameBuffer[8]<<8))))/10,(float(short(mFrameBuffer[9]|(mFrameBuffer[10]<<8))))/10,(float(short(mFrameBuffer[11]|(mFrameBuffer[12]<<8))))/10,(float(short(mFrameBuffer[13]|(mFrameBuffer[14]<<8))))/10,(float(short(mFrameBuffer[15]|(mFrameBuffer[16]<<8))))/10,(float(short(mFrameBuffer[17]|(mFrameBuffer[18]<<8))))/10);

                                //printf("ARM->ROS: %f,%f,%f,%f,%f,%f\n",float(mFrameBuffer[5]|mFrameBuffer[6]<<8)/10,float(mFrameBuffer[7]|mFrameBuffer[8]<<8)/10,float(mFrameBuffer[9]|mFrameBuffer[10]<<8)/10,float(mFrameBuffer[11]|mFrameBuffer[12]<<8)/10,float(mFrameBuffer[13]|mFrameBuffer[14]<<8)/10,float(mFrameBuffer[15]|mFrameBuffer[16]<<8)/10);
                            }
                        }
                        else
                        {
                            printf("???????????????\n");
                        }
                    }
                    else
                    {
                        printf("???????????????!\n");
                        print_hex(mFrameBuffer, mDataLength);

                    }
                    mDataLength = 0;
                }
            }
        }
        return ExitSuccess; // return success to continue
    }

    void CSDarmCommon::CheckJointNum(sdk_sagittarius_arm::SDKSagittariusArmConfig &config)
    {
        if(config.min_ang > config.max_ang)
        {
            ROS_WARN("Minimum angle must be greater than maxmum angle. Adjusting min_ang");
            config.min_ang = config.max_ang;
        }
    }

    void CSDarmCommon::UpdateConfig(sdk_sagittarius_arm::SDKSagittariusArmConfig &newConfig, uint32_t level)
    {
        CheckJointNum(newConfig);
        mConfig = newConfig;
    }

    void CSDarmCommon::DumpLaserMessage(sensor_msgs::LaserScan &msg)
    {
        ROS_DEBUG("Laser Message to send:");
        ROS_DEBUG("Header  frame_id: %s", msg.header.frame_id.c_str());
        //ROS_DEBUG("Header timestamp: %ld", msg.header.stamp);
        ROS_DEBUG("angle_min: %f", msg.angle_min);
        ROS_DEBUG("angle_max: %f", msg.angle_max);
        ROS_DEBUG("angle_increment: %f", msg.angle_increment);
        ROS_DEBUG("time_increment: %f", msg.time_increment);
        ROS_DEBUG("scan_time: %f", msg.scan_time);
        ROS_DEBUG("range_min: %f", msg.range_min);
        ROS_DEBUG("range_max: %f", msg.range_max);
    }
} // sdk_sagittarius_arm
