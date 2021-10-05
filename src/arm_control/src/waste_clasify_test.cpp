/*
简介：该文档作为主功能实现文件,按逻辑依次实现机械臂对指定目标的抓取
作者：xinnie@iflytek.com
日期：20191206
版本：V1
*/
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>
#include <arm_control/Move_Target_3d.h>
#include <arm_control/Get_Current_3d.h>
#include <arm_control/Control_Manipulator.h>
#include <arm_control/Get_Pose_In_Range.h>
#include <arm_control/Card.h>
#include <queue>
#include <arm_control/Get_Qcode_Location.h>
#include <iostream>
using namespace std;
#define GRIPPER_LENGTH 190
#include <ctime>
#include <Eigen/Dense>
using namespace Eigen;

ros::Subscriber sub_car_reach;
ros::Subscriber sub_car_location;
ros::Subscriber sub_mission;
ros::Subscriber sub_detection_result;
ros::ServiceClient client_move_arm;
ros::ServiceClient client_get_inrange;
ros::ServiceClient client_get_arm;
ros::ServiceClient client_use_pump;
ros::ServiceClient client_qcode_location;
ros::ServiceClient client_voice;
ros::Publisher pub_cmd_vel;
ros::Publisher pub_finish_grasped;

float stretch_upper = 346;
float stretch_lower = 50;
float height_upper = 200;
float height_lower = 1;
int place_type = 999;
struct Position {
    float x;
    float y;
    float z;
} position;

typedef std::pair<Position, int> xf_pair;
std::vector <xf_pair> grasp_list;
std::vector <xf_pair> inrange_grasp_list;

/**变量定义**/
int card_number = 0;
bool ret = false;
int grasp_type = 99;       //此项需修改为0
bool is_initial = false;   //机械臂是否在初始位置
bool is_car_reach = true; // 小车是否到达放置位置
Position object_location;
Position car_location;
float current_x = 0;
float current_y = 0;
float current_z = 0;
float test_y = 0;
bool action = false; //
float angle_z = 0;

Position DRY_POSE;
Position WET_POSE;
Position POISONOUS_POSE;
Position SAVE_POSE; //干垃圾位置

vector <Position> place_positions;

bool In_range(int mode, float pose_x, float pose_y, float pose_z) {
    if (mode == 1) {
        arm_control::Get_Current_3d srv_get;
        srv_get.request.mode = "xyz";
        if (client_get_arm.call(srv_get)) {
            if (srv_get.response.success == 1) {
                current_x = srv_get.response.x_s;
                current_y = srv_get.response.y_r;
                current_z = srv_get.response.z_h;
            }
        }

        pose_x += current_x;
        pose_y += current_y;
        pose_z = 5;
        arm_control::Get_Pose_In_Range srv_get_range;
        srv_get_range.request.x_s = pose_x;
        srv_get_range.request.y_r = pose_y;
        srv_get_range.request.z_h = pose_z;
        if (client_get_inrange.call(srv_get_range)) {
            if (srv_get_range.response.success == 1) {
                if (srv_get_range.response.in_range) {
                    return true;
                } else {
                    return false;
                }
            }
        }
    } else {
        arm_control::Get_Pose_In_Range srv_get_range;
        srv_get_range.request.x_s = pose_x;
        srv_get_range.request.y_r = pose_y;
        srv_get_range.request.z_h = pose_z;
        if (client_get_inrange.call(srv_get_range)) {
            if (srv_get_range.response.success == 1) {
                if (srv_get_range.response.in_range) {
                    return true;
                } else {
                    return false;
                }
            }
        }
    }
}

//此函数应该是重复待机执行的,接受订单
void Order_Callback(const arm_control::CardConstPtr &msg) {
    int a = msg->data; //有几个？
    vector<xf_pair>().swap(grasp_list);
    if (is_initial) {
        //将图像捕捉到的所有卡片全部加入
        for (int i = 0; i < a; i++) {
            xf_pair grasp;    
            if ((int) msg->pose[4 * i] == 1 || (int) msg->pose[4 * i] == 3) {
                grasp.second = 2;
            } else if ((int) msg->pose[4 * i] == 0 || (int) msg->pose[4 * i] == 6) {
                grasp.second = 3;
            } else if ((int) msg->pose[4 * i] == 2 || (int) msg->pose[4 * i] == 4) {
                grasp.second = 1;
            } else if ((int) msg->pose[4 * i] == 5 || (int) msg->pose[4 * i] == 7) {
                grasp.second = 4;
            }
            grasp.first.x = msg->pose[4 * i + 1];
            grasp.first.y = msg->pose[4 * i + 2];
            grasp.first.z = msg->pose[4 * i + 3];

            grasp_list.push_back(grasp);
        }
    }
}

//99为一直抓直到画面中没有垃圾 1-4为只抓命令词要求的即干湿害回,均放置在小车上
void Grasp_Type_Callback(const std_msgs::Int8ConstPtr &msg1) {
    grasp_type = msg1->data;
}

void Whether_Car_Rearch_Callback(const std_msgs::Int8ConstPtr &msg2) //3为到达放置点
{
    if (msg2->data == 3) {
        is_car_reach = true;
    }
}

bool move_to_target_pose(string mode = "xyz", float x = 200.0, float y = 0.0, float z = 10.0, float speed = 30.0) {
    arm_control::Get_Current_3d srv_get;
    srv_get.request.mode = "xyz";
    if (client_get_arm.call(srv_get)) {
        if (srv_get.response.success == 1) {
            current_x = srv_get.response.x_s;
            current_y = srv_get.response.y_r;
            current_z = srv_get.response.z_h;
        }
    }
    angle_z = atan2(current_y, current_x);
    Eigen::Matrix<float, 4, 4> m_robot;
    m_robot << cos(angle_z),-sin(angle_z),0,current_x,sin(angle_z),cos(angle_z),0,current_y,0,0,1,current_z,0,0,0,1;
    Eigen::Matrix<float, 4, 1> m_position;
    m_position << x,y,z,1;
    Eigen::Matrix<float, 4, 1> m_result = m_robot * m_position;
    arm_control::Move_Target_3d srv;
    srv.request.direction = mode;
    srv.request.x = (int) (m_result(0,0));
    srv.request.y = (int) (m_result(1,0));
    srv.request.z = 5;
    srv.request.s = 0;
    srv.request.r = 0;
    srv.request.h = 0;
    srv.request.is_relative = 0;
    srv.request.vel = speed;
    srv.request.move_mode = "MOVJ";
    if (client_move_arm.call(srv)) {
        if (srv.response.success == 1) {
            cout << "到达目标位置" << endl;
            return true;
        } else {
            cout << "Fail reason:" << srv.response.message << endl;
            return false;
        }
    } else {
        ROS_ERROR("Failed to call service [move_to_target_xyz]");
        return false;
    }
}

bool move_to_target(string mode = "xyz", float x = 200.0, float y = 0.0, float z = 10.0, float speed = 40.0) {
    arm_control::Move_Target_3d srv;
    srv.request.direction = mode;
    srv.request.x = (int) x;
    srv.request.y = (int) y;
    srv.request.z = (int) z;
    srv.request.s = 0;
    srv.request.r = 0;
    srv.request.h = 0;
    srv.request.is_relative = 0;
    srv.request.vel = speed;
    srv.request.move_mode = "MOVJ";
    if (client_move_arm.call(srv)) {
        //ROS_INFO("[call result]: %s,\t[fail_reason]:%s", srv.response.result.c_str(), srv.response.fail_reason.c_str());
        if (srv.response.success == 1) {
            return true;
        } else {
            cout << "Fail reason:" << srv.response.message << endl;
            return false;
        }
    } else {
        ROS_ERROR("Failed to call service [move_to_target_xyz]");
        return false;
    }
}

bool use_pump(int if_use) {
    arm_control::Control_Manipulator srv;
    //srv.request.mode = "pump";//last version for black_arm 
    if (if_use) {
        srv.request.used = true;
    } else {
        srv.request.used = false;
    }
    if (client_use_pump.call(srv)) {
        if (srv.response.success == 1) {
            return true;
        } else {
            return false;
        }
    } else {
        return false;
    }
}

void initial_parmers() {
    is_car_reach = false;
    grasp_type = 0;
    car_location.x = 100;
    car_location.y = 200;
    car_location.z = 40;
    use_pump(0);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "waste_clasify_test");
    ros::NodeHandle n;
    printf("----------------等待服务中\n");
    ros::service::waitForService("/arm/get_current_xyz");
    ros::service::waitForService("/arm/move_to_target_xyz");
    ros::service::waitForService("arm/control_manipulator");
    printf("----------------检测到服务，准备发送请求\n");
    client_get_arm = n.serviceClient<arm_control::Get_Current_3d>("/arm/get_current_xyz");
    client_get_inrange = n.serviceClient<arm_control::Get_Pose_In_Range>("/arm/get_pose_in_range");
    client_move_arm = n.serviceClient<arm_control::Move_Target_3d>("/arm/move_to_target_xyz");
    client_use_pump = n.serviceClient<arm_control::Control_Manipulator>("/arm/control_manipulator");
    sub_detection_result = n.subscribe("/card/detection_result", 1, Order_Callback); //订阅检测结果
    sub_mission = n.subscribe("/mission", 1, Grasp_Type_Callback);                   //订阅任务信息
    //是否已到达
    sub_car_reach = n.subscribe("/ucar_msg", 1, Whether_Car_Rearch_Callback); //订阅小车到达信息
    pub_finish_grasped = n.advertise<std_msgs::Int8>("/arm_msg", 1);          //发布已放置好的话题
    ros::AsyncSpinner spinner(4);
    spinner.start();
    initial_parmers();

    while (ros::ok()) {
        /** [1] 机械臂回到初始位置**/
        ret = move_to_target("xyz", 200, 0, 120);
        sleep(1);

        if (ret) {
            is_initial = true;
        }
        std::vector <xf_pair> grasp_list_current(grasp_list);
        /**判断有无让抓取,这个是首要的前提,此处订单是布尔量,待修改**/
        //cout << "the size of list:" << grasp_list.size() << endl;
        if (grasp_list_current.size() > 0 && grasp_type != 0) //检测完后需要对列表进行一个修正，根据类型修整。
        {
            std::vector<xf_pair>::iterator iter;

            for (iter = grasp_list_current.begin(); iter != grasp_list_current.end();) {
                if (iter->second != grasp_type) {
                    iter = grasp_list_current.erase(iter);
                } else {
                    iter++;
                }
            }

            std::vector<xf_pair>::iterator iter1;
            for (iter1 = grasp_list_current.begin(); iter1 != grasp_list_current.end();) {
                if (!In_range(1, iter1->first.x, iter1->first.y, iter1->first.z)) {
                    grasp_list_current.erase(iter1);
                } else {
                    ++iter1;
                }
            }

            card_number = grasp_list_current.size();
            if (grasp_list_current.size() > 0) {
                for (int j = 0; j < card_number; j++) {
                    object_location.x = grasp_list_current[j].first.x;
                    object_location.y = grasp_list_current[j].first.y;
                    object_location.z = grasp_list_current[j].first.z;
                    place_type = grasp_list_current[j].second;
                }
            }

            ros::spinOnce();
            /********************************************************************************/
            //如果机械臂回到初始位置
            if (is_initial  && card_number > 0) {  //is_initial && is_car_reach && card_number
                is_initial = false;
                card_number--;
                printf("----------------[2] 到达卡片上方\n");
                cout << "object_location.x:" << object_location.x << "object_location.y" << object_location.y << "z:" << object_location.z << endl;
                ret = move_to_target_pose("xyz", object_location.x, object_location.y, object_location.z); //10); //卡片上方
                if (ret) {
                    printf("----------------[3] 到达卡片位置\n");
                    ret = move_to_target("z", 0, 0, 5);
                    if (ret) {
                        printf("----------------[4] 打开吸泵\n");
                        ret = use_pump(1);
                        sleep(1);
                        if (ret) {
                            printf("----------------[5] 到达卡片上方\n");
                            ret = move_to_target("z", 0, 0, 115);
                            if (ret && In_range(0, car_location.x, car_location.y, car_location.z)) {
                                printf("----------------[6] 到达机械臂上方准备检测机械臂位置\n");
                                ret = move_to_target("xyz", car_location.x, car_location.y, car_location.z + 10);
                                if (ret) {
                                    //your code :修正car位置";
                                    printf("----------------[7] 到达放置位置\n");
                                    ret = move_to_target("z", 0, 0, car_location.z);
                                    if (ret) {
                                        printf("----------------[8] 关闭吸泵\n");
                                        ret = use_pump(0);
                                        if (ret) {
                                            printf("----------------[9] 回到机械臂上方位置\n");
                                            ret = move_to_target("z", 0, 0, car_location.z + 20);
                                            printf("----------------[10] 发布放置完成指令\n");
                                            if (card_number == 0) {
                                                std_msgs::Int8 is_place_finished;
                                                is_place_finished.data = 1;
                                                pub_finish_grasped.publish(is_place_finished);
                                                initial_parmers();
                                            }
                                            if (ret) {
                                                printf("----------------[11] 回到初始位置\n");
                                                ret = move_to_target("xyz", 200, 0, 120); //初始位置
                                                if (ret) {
                                                    //grasp_list.clear();
                                                    vector<xf_pair>().swap(grasp_list_current);

                                                    cout << "whether empty:" << grasp_list_current.size() << endl;
                                                    is_initial = true;

                                                    sleep(1);
                                                    continue;
                                                } else {
                                                    printf("[error]:error in [11] 回到初始位置\n");
                                                    vector<xf_pair>().swap(grasp_list_current);
                                                    continue;
                                                }
                                            } else {
                                                printf("[error]:error in [9] 回到小车上方位置\n");
                                                vector<xf_pair>().swap(grasp_list_current);
                                                continue;
                                            }
                                        } else {
                                            printf("[error]:error in [8] 关闭吸泵\n");
                                            vector<xf_pair>().swap(grasp_list_current);
                                            continue;
                                        }
                                    } else {
                                        printf("[error]:error in [7] 修正小车位置,并到达其上方\n");
                                        vector<xf_pair>().swap(grasp_list_current);
                                        continue;
                                    }
                                } else {
                                    printf("[error]:error in [6] 到达小车上方准备检测小车位置\n");
                                    vector<xf_pair>().swap(grasp_list_current);
                                    continue;
                                }
                            } else {
                                printf("[error]:error in [5] 到达卡片上方或小车不在机械臂可到达范围内\n");
                                vector<xf_pair>().swap(grasp_list_current);
                                continue;
                            }
                        } else {
                            printf("[error]:error in [4] 到达卡片上方\n");
                            vector<xf_pair>().swap(grasp_list_current);
                            continue;
                        }
                    } else {
                        printf("[error]:error in [3] 到达卡片位置\n");
                        vector<xf_pair>().swap(grasp_list_current);
                        continue;
                    }
                } else {
                    printf("[error]:error in [2] 到达卡片上方\n");
                    vector<xf_pair>().swap(grasp_list_current);
                    continue;
                }
            } else {
                /*如果不在准备状态,则开始采取措施,让其到达稳定状态*/
                sleep(1);
                continue;
            }
        } else {
            continue;
        }

    }
//  ros::waitForShutdown();
    return 0;
}
