#include<opencv2/core.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc.hpp>
using namespace cv;
#include<iostream>
#include<string>
using namespace std;
//输入图像
Mat img;
//灰度值归一化
Mat bgr;
//HSV图像
Mat hsv;
//色相
int hmin = 0;
int hmin_Max = 360;
int hmax = 360;
int hmax_Max = 360;
//饱和度
int smin = 70;
int smin_Max = 255;
int smax = 255;
int smax_Max = 255;
//亮度
int vmin = 60;
int vmin_Max = 255;
int vmax = 250;
int vmax_Max = 255;
//显示原图的窗口
string windowName = "src";
//输出图像的显示窗口
string dstName = "dst";
//输出图像
Mat dst;
string image_write_path = "";
//回调函数
void callBack(int, void*)
{
	//输出图像分配内存
	dst = Mat::zeros(img.size(), CV_32FC3);
	//掩码
	Mat mask;
	inRange(hsv, Scalar(hmin, smin / float(smin_Max), vmin / float(vmin_Max)), Scalar(hmax, smax / float(smax_Max), vmax / float(vmax_Max)), mask);
	//只保留
	for (int r = 0; r < bgr.rows; r++)
	{
		for (int c = 0; c < bgr.cols; c++)
		{
			if (mask.at<uchar>(r, c) == 255)
			{
				dst.at<Vec3f>(r, c) = bgr.at<Vec3f>(r, c);
			}
		}
	}
	//输出图像
	imshow(dstName, dst);
	//保存图像
	dst.convertTo(dst, CV_8UC3, 255.0, 0);
	imwrite(image_write_path, dst);
}
int main(int argc, char*argv[])
{
	//输入图像
	image_write_path = string(argv[1]).substr(0, string(argv[1]).find_last_of('/') + 1) + "HSV_inRange.jpg";
	
	img = imread(argv[1], IMREAD_COLOR);
	if (!img.data || img.channels() != 3)
		return -1;
	imshow(windowName, img);
	//彩色图像的灰度值归一化
	img.convertTo(bgr, CV_32FC3, 1.0 / 255, 0);
	//颜色空间转换
	cvtColor(bgr, hsv, COLOR_BGR2HSV);
	//定义输出图像的显示窗口
	namedWindow(dstName, WINDOW_GUI_EXPANDED);
	//调节色相 H
	createTrackbar("hmin", dstName, &hmin, hmin_Max, callBack);
	createTrackbar("hmax", dstName, &hmax, hmax_Max, callBack);
	//调节饱和度 S
	createTrackbar("smin", dstName, &smin, smin_Max, callBack);
	createTrackbar("smax", dstName, &smax, smax_Max, callBack);
	//调节亮度 V
	createTrackbar("vmin", dstName, &vmin, vmin_Max, callBack);
	createTrackbar("vmax", dstName, &vmax, vmax_Max, callBack);
	callBack(0, 0);
	waitKey(0);
	return 0;
}
