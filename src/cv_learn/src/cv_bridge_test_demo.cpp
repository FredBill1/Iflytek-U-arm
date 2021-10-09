#include <cv_bridge/cv_bridge.h>              //引入cv_bridge库用于图像转化
#include <image_transport/image_transport.h>  //引入import_transport库用于图像传输
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>  //引入opencv中的人机图像接口单元
#include <opencv2/imgproc/imgproc.hpp>  //引入opencv中的图像处理单元

static const std::string OPENCV_WINDOW = "Image window";
class ImageConverter {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

 public:
    // 构造函数
    ImageConverter() : it_(nh_) {
        // 订阅“/camera/image_raw””主题，并在回调函数imageCb进行操作
        image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &ImageConverter::imageCb, this);
        // 发布“/image_converter/output_video”话题
        image_pub_ = it_.advertise("/image_converter/output_video", 1);
        // 窗口初始化
        cv::namedWindow(OPENCV_WINDOW);
    }
    // 析构函数
    ~ImageConverter() {
        //释放内存前，销毁窗口
        cv::destroyWindow(OPENCV_WINDOW);
    }
    // 回调函数，接收sensor_msgs::ImageConstPtr类型的参数
    void imageCb(const sensor_msgs::ImageConstPtr &msg) {
        cv_bridge::CvImagePtr cv_ptr;  //声明一个指针指向图像
        try {  // 调用cv_bridge获取将ROS图像数据转化为Opencv的图像格式，其中tocvcopy复制从ROS消息的图像数据，可以自由修改返回的图像，在本节点中，数据编码格式为带有颜色信息且顺序是BGR。
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        //在图像上画一个圆
        if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
            cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255, 0, 0));
        // 在窗口中进行显示
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        //	cv::waitKey(3);
        // 发布图像视频流
        image_pub_.publish(cv_ptr->toImageMsg());
    }
};
int main(int argc, char **argv) {
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    while (1) {
        ros::spinOnce();
        cv::waitKey(1);
    }
    //    ros::spin();

    return 0;
}
