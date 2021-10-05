#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h> //引入import_transport库用于图像传输
#include <cv_bridge/cv_bridge.h>             //引入cv_bridge库用于图像转化
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp> //引入opencv中的图像处理单元
#include <opencv2/highgui/highgui.hpp> //引入opencv中的人机图像接口单元
#include <geometry_msgs/Twist.h>

using namespace cv;

class ColorSegemation
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher object_pub_;
    Mat src;
    std::vector<std::vector<Point>> contours;
    std::vector<Vec4i> hireachy;
    Rect rect;
    Point2f center;
    float radius;
    float Max_R = 50.0;
    RotatedRect rrt;

public:
    // 构造函数
    ColorSegemation() : it_(nh_)
    {
        // 订阅“/camera/image_raw””主题，并在回调函数imageCb进行操作
        image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
                                   &ColorSegemation::imageCb, this);
        // 发布“/image_converter/output_video”话题
        object_pub_ = nh_.advertise<geometry_msgs::Twist>("UV_location", 1);
    }
    // 析构函数
    ~ColorSegemation()
    {
    }

    // 回调函数，接收sensor_msgs::ImageConstPtr类型的参数
    void imageCb(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr; //声明一个指针指向图像
        try
        {
            // 调用cv_bridge获取将ROS图像数据转化为Opencv的图像格式，其中tocvcopy复制从ROS消息的图像数据，可以自由修改返回的图像，在本节点中，数据编码格式为带有颜色信息且顺序是BGR。
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        if (cv_ptr->image.rows > 0 && cv_ptr->image.cols > 0)
        {
            src = cv_ptr->image;
            // 将图像从RGB空间转换到HSV空间
            Mat src_hsv = Mat(src.size(), CV_8UC3);
            cvtColor(src, src_hsv, cv::COLOR_BGR2HSV);

            // 利用hsv分量阈值进行二值化分割
            Mat out_binary = Mat(src.size(), CV_8UC1);
            inRange(src_hsv, Scalar(35, 43, 46), Scalar(77, 255, 255), out_binary);
            /*green[(35,43,46)-(77,255,255)]  red[(156,43,46)-(180,255,255)]  pink[(125,43,46)-(155,255,255)]*/

            // 利用形态学运算消除噪声
            int g_nstructElementSize = 3;
            Mat element = getStructuringElement(MORPH_RECT, Size(2 * g_nstructElementSize + 1, 2 * g_nstructElementSize + 1), Point(g_nstructElementSize, g_nstructElementSize));
            morphologyEx(out_binary, out_binary, MORPH_OPEN, element);

            //边缘检测
            Canny(out_binary, out_binary, 3, 9, 3);

            // 轮廓查找，寻找最外侧轮廓
            findContours(out_binary, contours, hireachy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));

            //轮廓筛选，寻找最大轮廓，目标物一般为最大轮廓
            if (contours.size() > 0)
            {
                double maxArea = 0;
                for (int i = 0; i < contours.size(); i++)
                {
                    double area = contourArea(contours[static_cast<int>(i)]);
                    if (area > maxArea)
                    {
                        maxArea = area;
                        rect = boundingRect(contours[static_cast<int>(i)]);
                        rrt = minAreaRect(contours[static_cast<int>(i)]);
                        minEnclosingCircle(contours[static_cast<int>(i)], center, radius);
                    }
                }
                if (radius > Max_R)
                {
                    //框选住目标物体，并指出轮廓质心位置
                    Point2f pts[4];
                    rrt.points(pts);
                    for (int i = 0; i < 4; i++)
                    {
                        line(src, pts[i % 4], pts[(i + 1) % 4], Scalar(255, 0, 0), 2, 8, 0);
                    }
                    Point2f cpt = rrt.center;
                    circle(src, cpt, 2, Scalar(0, 0, 255), 2, 8, 0);
                    //circle(src, Point(center.x, center.y), (int)radius, Scalar(0, 255, 0), 2);
                    char tam[100];
                    sprintf(tam, "(%0.0f,%0.0f)", center.x, center.y);
                    cv::putText(src, tam, Point(center.x, center.y), FONT_HERSHEY_SIMPLEX, 0.6, cvScalar(0, 0, 0), 2);
                    // 发布物体在图像坐标系中的位置
                    geometry_msgs::Twist object_position;
                    object_position.linear.x = center.x;
                    object_position.linear.y = center.y;
                    object_pub_.publish(object_position);
                }
            }

            imshow("out", src);
            //waitKey(0);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "color_object_segemation");
    ColorSegemation ic;
    while (ros::ok())
    {
        ros::spinOnce();
        waitKey(1);
    }
    return 0;
}
