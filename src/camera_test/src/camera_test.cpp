#include <ros/ros.h>

#include <iostream>
#include <opencv2/opencv.hpp>
int main(int argc, char **argv) {
    ros::init(argc, argv, "camera_test");
    cv::VideoCapture capture(0);  // 0相机设备的标号为０，即相机设备为编号”video0”的设备
    if (!capture.isOpened()) {    //判断相机是否被打开，如果指定相机不存在就会打开失败
        std::cout << "Camera Read Failed !" << std::endl;
        return -1;
    }
    cv::Mat frame;                  //定义一个图像变量
    cv::namedWindow("video test");  //定义一个用于图像显示的窗口
    while (1) {
        capture >> frame;  //采集一帧图像赋值给图像变量
        /*****
        自定义图像处理过程与结果发布程序段
        *****/
        imshow("video test", frame);   //图像显示
        if (cv::waitKey(30) == 'q') {  //等待时间，如果按Esc键则退出图像采集和处理流程
            cv::destroyWindow("video test");
            capture.release();
        }
    }
    ros::spin();
    return 0;
}
