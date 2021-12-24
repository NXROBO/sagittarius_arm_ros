开启摄像头
roslaunch sagittarius_object_color_detector usb_cam.launch 


颜色筛选
1、配置 config/vision_config.yaml 中红绿蓝HSV值，可通过 tool/para_test 提取获取值
2、开启摄像头和识别器
roslaunch sagittarius_object_color_detector object_detect_hsv.launch
3、执行抓取动作（所有颜色分类，一步到位）
rosrun sagittarius_object_color_detector catch_cube.py all false
