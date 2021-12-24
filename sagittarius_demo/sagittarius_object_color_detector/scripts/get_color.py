#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from sagittarius_object_color_detector.srv import *

if __name__ == "__main__":
    # 初始化ROS节点
    rospy.init_node('auto_calibration')
    while(not rospy.is_shutdown()):
        try:
            # 请求服务
            rospy.wait_for_service('/object_detect')

            detect_object_service = rospy.ServiceProxy('/object_detect', DetectObjectSrv)
            print(detect_object_service(DetectObjectSrvRequest.ALL_OBJECT))
            rospy.sleep(0.1)
        except rospy.ROSException:
            rospy.loginfo("Timeout waiting for image data.")

