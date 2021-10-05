#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h> //引入import_transport库用于图像传输
#include <cv_bridge/cv_bridge.h>             //引入cv_bridge库用于图像转化
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp> //引入opencv中的图像处理单元
#include <opencv2/highgui/highgui.hpp> //引入opencv中的人机图像接口单元
#include <geometry_msgs/Twist.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <vector>
using namespace cv;
using namespace std;
// 定义一个结构体用于存放角点信息
typedef struct PointInfo
{
    double Direction;
    double Magnitude;
    double MagnitudeN;
    double DerivativeX;
    double DerivativeY;
} ptin;

long contoursLength = 0;
double magnitudeTemp = 0;
float minScore = 1.0;
float greediness = 0.8;
std::vector<double> resultScore_;
std::vector<int> resultX_;
std::vector<int> resultY_;

Mat src1, src;
std::vector<std::vector<Point> > contours;
std::vector<Vec4i> hireachy;
Rect rect;
Point2f center;
float radius;
vector<vector<ptin> > contoursInfo;
// 提取相对坐标位置
vector<vector<Point> > contoursRelative;

void caclute_muban()
{
    printf("in caclute_muban");
    Mat tpl = imread("/home/nie/4.png", -1);
    if (tpl.empty())
    {
        printf("could not load images...\n");
    }
    //模板处理
    Mat gray;
    Mat binary;
    //色彩空间转换，彩色转到灰度空间
    cvtColor(tpl, gray, cv::COLOR_BGR2GRAY, 0);
    //边缘检测，阈值设置为100和800
    Canny(tpl, binary, 100, 800);
    // 轮廓检测
    vector<vector<Point> > contours;
    vector<Vec4i> hireachy;
    findContours(binary, contours, hireachy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(-1, -1));
    Mat gx, gy;
    Sobel(gray, gx, CV_32F, 1, 0);
    Sobel(gray, gy, CV_32F, 0, 1);
    Mat magnitude, direction;
    cartToPolar(gx, gy, magnitude, direction);

    int originx = contours[0][0].x;
    int originy = contours[0][0].y;

    // 开始提取两个量，第一个量是轮廓上的每个点相对于第一个点（第０个轮廓上的第０个点）的位置
    // 第二个量是每个点的属性信息（ｘ,y方向梯度，梯度）
    for (int i = 0; i < contours.size(); i++)
    {
        //第ｉ个轮廓上有多少个点
        int n = contours[i].size();
        contoursLength += n;
        contoursInfo.push_back(vector<ptin>(n));
        vector<Point> points(n);

        for (int j = 0; j < n; j++)
        {
            int x = contours[i][j].x;
            int y = contours[i][j].y;
            //相对于第一个点的位置
            points[j].x = x - originx;
            points[j].y = y - originy;
            ptin pointInfo;
            pointInfo.DerivativeX = gx.at<float>(y, x); //x方向梯度
            pointInfo.DerivativeY = gy.at<float>(y, x); //ｙ方向梯度
            magnitudeTemp = magnitude.at<float>(y, x);  //梯度
            pointInfo.Magnitude = magnitudeTemp;
            if (magnitudeTemp != 0)
                pointInfo.MagnitudeN = 1 / magnitudeTemp;
            contoursInfo[i][j] = pointInfo;
        }
        //相对点的位置信息
        contoursRelative.push_back(points);
    }
}

// 回调函数，接收sensor_msgs::ImageConstPtr类型的参数
void ImageRead()
{
    printf("in ImageRead");
    Mat src1 = imread("/home/nie/2.jpg", -1);
    if (src1.empty())
    {
        printf("could not load images...\n");
    }

    if (src1.rows > 0 && src1.cols > 0)
    {
        //src1 = cv_ptr->image;
        // 计算目标图像梯度
        Mat grayImage;
        //色彩空间转换
        cout << "channels" << src1.channels() << endl;
        cvtColor(src1, grayImage, COLOR_BGR2GRAY, 0);

        imshow("gray", grayImage);
        //x与ｙ方向梯度
        Mat gradx, grady;
        Sobel(grayImage, gradx, CV_32F, 1, 0);
        Sobel(grayImage, grady, CV_32F, 0, 1);
        Mat mag, angle;
        cartToPolar(gradx, grady, mag, angle);

        long totalLength = contoursLength;
        double nMinScore = minScore / totalLength; // normalized min score
        double nGreediness = (1 - greediness * minScore) / (1 - greediness) / totalLength;

        double partialScore = 0;
        double resultScore = 0;
        int resultX = 0;
        int resultY = 0;
        double start = (double)getTickCount();
        for (int row = 0; row < grayImage.rows; row++)
        {
            for (int col = 0; col < grayImage.cols; col++)
            {
                double sum = 0;
                long num = 0;
                for (int m = 0; m < contoursRelative.size(); m++)
                {
                    for (int n = 0; n < contoursRelative[m].size(); n++)
                    {
                        num += 1;
                        int curX = col + contoursRelative[m][n].x;
                        int curY = row + contoursRelative[m][n].y;
                        if (curX < 0 || curY < 0 || curX > grayImage.cols - 1 || curY > grayImage.rows - 1)
                        {
                            continue;
                        }

                        // 目标边缘梯度
                        double sdx = gradx.at<float>(curY, curX);
                        double sdy = grady.at<float>(curY, curX);

                        // 模板边缘梯度
                        double tdx = contoursInfo[m][n].DerivativeX;
                        double tdy = contoursInfo[m][n].DerivativeY;

                        // 计算匹配
                        if ((sdy != 0 || sdx != 0) && (tdx != 0 || tdy != 0))
                        {
                            double nMagnitude = mag.at<float>(curY, curX);
                            if (nMagnitude != 0)
                                sum += (sdx * tdx + sdy * tdy) * contoursInfo[m][n].MagnitudeN / nMagnitude;
                        }

                        // 任意点score之和必须大于最小阈值
                        partialScore = sum / num;
                        if (partialScore < min((minScore - 1) + (nGreediness * num), nMinScore * num))
                            break;
                        sum = 0;
                        /**/
                    }
                }
                // 保存匹配起始点
                if (partialScore > resultScore)
                {
                    resultScore = partialScore;
                    resultX = col;
                    resultY = row;
                    resultScore_.push_back(resultScore);
                    resultX_.push_back(resultX);
                    resultY_.push_back(resultY);
                }
            }
        }

        std::vector<double>::iterator biggest = std::max_element(resultScore_.begin(), resultScore_.end());

        cout << "Max element is " << *biggest << " at position " << std::distance(resultScore_.begin(), biggest) << endl;
        int result_X = resultX_.at(std::distance(resultScore_.begin(), biggest));
        int result_Y = resultY_.at(std::distance(resultScore_.begin(), biggest));
        cout << "result_X:" << result_X << "result_Y" << result_Y << endl;
        Point offset_point(result_X, result_Y);
        float time = (((double)getTickCount() - start)) / getTickFrequency();
        printf("edge template matching time : %.2f seconds\n", time);

        drawContours(src1, contoursRelative, -1, Scalar(255, 0, 0), 2, 8, Mat(), INT_MAX, offset_point);
        imshow("edge-match", src1);
        
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "match_object_segemation");
    caclute_muban();
    ImageRead();
    cv::waitKey(0);
    return 0;
}
