/*******************************************************
 This contents of this file may be used by anyone
 for any reason without any conditions and may be
 used as a starting point for your own applications
 which use HIDAPI.
********************************************************/
#include "user_interface.h"
//#include <sys/io.h>
#include <sys/types.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <xf_mic_api/Pcm_msg.h>
#include <xf_mic_api/Get_Wav_List_srv.h>
#include <xf_mic_api/Play_Target_Wav_srv.h>
#include <xf_mic_api/Delete_Target_Wav_srv.h>
#include <xf_mic_api/Adjust_Voice_srv.h>
#include <xf_mic_api/Stop_To_Play_srv.h>

#define _GNU_SOURCE
#define PCM_MSG_LEN 512

std::string awake_angle_topic = "/mic/awake/angle";
std::string record_topic = "/mic/record/whethet_start";
std::string pcm_topic = "/mic/pcm/deno";
std::string package_path = " ";

//int if_awake = 0;
ros::Publisher pub_pcm;
ros::Publisher pub_awake_angle;
ros::Subscriber sub_record_start;
ros::ServiceClient client;

using namespace std;

void GetFileNames(string path, vector<string> &filenames)
{
	DIR *pDir;
	struct dirent *ptr;
	if (!(pDir = opendir(path.c_str())))
		return;
	while ((ptr = readdir(pDir)) != 0)
	{
		if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0)
			filenames.push_back(path + "/" + ptr->d_name);
	}
	closedir(pDir);
}

/*
content:srv 获取指定目录中所有后缀为wav的音频文件
data :20200407 AM
*/
bool Get_wav_list(xf_mic_api::Get_Wav_List_srv::Request &req,
				  xf_mic_api::Get_Wav_List_srv::Response &res)
{
	if (req.get_list == 1) //如果是1表示获取音频列表
	{
		/*查看该目录下所有的后缀为wav的文件，并发布出去*/
		string wav_path = package_path + "/audio";
		vector<string> filenames_list;
		GetFileNames(wav_path, filenames_list);
		if (filenames_list.size() > 0)
		{
			for (int i = 0; i < filenames_list.size(); i++)
			{
				int location1 = filenames_list[i].find("audio",0);
                int location2 = filenames_list[i].find(".wav",0);
				res.wav_list.push_back(filenames_list[i].substr(location1+6,location2-location1-6));
			}
			res.number = filenames_list.size();
			res.result = "ok";
			res.fail_reason = "";
		}
		else
		{
			res.number = 0;
			res.result = "fail";
			res.fail_reason = "no_wavs error";
			res.wav_list.push_back(" ");
		}
		ROS_INFO("send result to client,finish to load list\n");
	}
	return true;
}

/*
content: 判断要删除的文件是否存在，若存在则删除
data:20200407 AM
*/
bool Delete_Target_wav(xf_mic_api::Delete_Target_Wav_srv::Request &req,
					   xf_mic_api::Delete_Target_Wav_srv::Response &res)
{
	string delete_string = package_path + "/audio/" + req.filename+".wav";
	if (remove(delete_string.c_str()) == 0)
	{
		res.result = "ok";
		cout << "删除成功" << endl;
	}
	else
	{
		res.result = "false";
		res.fail_reason = "file_not_exist error";
		cout << "删除失败" << endl;
	}
	ROS_INFO("send result to client,finish to delete target wav\n");
	return true;
}

/*
content:srv callback,play wav
data :20200407 AM
*/
// bool Play_Wav(xf_mic_api::Play_Target_Wav_srv::Request &req,
// 			  xf_mic_api::Play_Target_Wav_srv::Response &res)
// {
// 	string adjust_voice = "pactl set-sink-volume 1 " + to_string(90)+"%";
// 	system(adjust_voice.c_str());

// 	string filename = package_path + "/audio/" + req.filename + ".wav";
// 	string playname = "play " + filename;
// 	// if (req.voice != 0) //如果设置了音量
// 	// {
// 	// 	//调整本地音频
// 	// 	string target_voice = to_string(req.voice / 100.0);
// 	// 	string adjust_wav = "sox -v" + target_voice + filename + " " + filename;
// 	// 	system(adjust_wav.c_str());
// 	// }
// 	system(playname.c_str());
// 	res.result = "ok";
// 	res.fail_reason = " ";
// 	ROS_INFO("send result to client,finish to play\n");
// 	return true;
// }

// /*
// content:srv callback,stop play wav
// data :20200407 AM
// */
// bool Stop_Play(xf_mic_api::Stop_To_Play_srv::Request &req,
// 			   xf_mic_api::Stop_To_Play_srv::Response &res)
// {
// 	if (req.stop_play == 1)
// 	{
// 		string adjust_voice = "pactl set-sink-volume 1 "  + to_string(0)+"%";
// 		system(adjust_voice.c_str());
// 		ROS_INFO("停止播放\n");
// 		//string stopplay = "p"; //如何在终端输入p
// 		//system(stopplay.c_str());
// 		res.result = "ok";
// 		res.fail_reason = " ";
// 	}

// 	ROS_INFO("send result to client,stop to play\n");
// 	return true;
// }

bool Play_Wav(xf_mic_api::Play_Target_Wav_srv::Request &req,
			  xf_mic_api::Play_Target_Wav_srv::Response &res)
{
	// string adjust_voice = "pactl set-sink-volume 1 " + to_string(90)+"%";
	// system(adjust_voice.c_str());

	string filename = package_path + "/audio/" + req.filename + ".wav";
	string playname = "mplayer " + filename;

	system(playname.c_str());
	res.result = "ok";
	res.fail_reason = " ";
	ROS_INFO("send result to client,finish to play\n");
	return true;
}

/*
content:srv callback,stop play wav
data :20200407 AM
*/

bool Stop_Play(xf_mic_api::Stop_To_Play_srv::Request &req,
			   xf_mic_api::Stop_To_Play_srv::Response &res)
{
	if (req.stop_play == 1)
	{
		string stop_play = "killall -9 mplayer";
		system(stop_play.c_str());
		ROS_INFO("停止播放\n");
		res.result = "ok";
		res.fail_reason = " ";
	}

	ROS_INFO("send result to client,stop to play\n");
	return true;
}



/*
content:srv 设定系统音量，0表示静音，100表示音量最大。
data :20200407 AM
*/
bool Adjust_Voice(xf_mic_api::Adjust_Voice_srv::Request &req,
				  xf_mic_api::Adjust_Voice_srv::Response &res)
{
	
        if(req.voice==101)
        {
           string adjust_voice = "pactl set-sink-volume @DEFAULT_SINK@ +10%";
           system(adjust_voice.c_str());
           res.result = "ok";
           
        }
        else if(req.voice==102)
        {
            string adjust_voice = "pactl set-sink-volume @DEFAULT_SINK@ -10%";
            system(adjust_voice.c_str());
            res.result = "ok";
        }
        else if(req.voice>=0 && req.voice<=100)
        {
           string adjust_voice = "pactl set-sink-volume @DEFAULT_SINK@ "+ to_string(req.voice)+"%";
           system(adjust_voice.c_str());
            res.result = "ok";
        }
	else
        {
          res.result = "fail";
          res.fail_reason = "input_value_error";
        }
	ROS_INFO("send result to client,finish to adjust voice\n");
	return true;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "xf_audio_play");
	ros::NodeHandle n("~");
	ROS_INFO("audio server start...\n");
	n.param("source_path", package_path, std::string("./"));
	/*　srv　获取当前音频列表,request:"get_wav_list",answer:vector<string>　*/
	ros::ServiceServer service_get_wav_list = n.advertiseService("get_wav_list_srv", Get_wav_list);

	/*　srv　删除指定的音频文件,request:"get_wav_list",answer:vector<string>　*/
	ros::ServiceServer service_delete_target_wav = n.advertiseService("delete_target_wav_srv", Delete_Target_wav);

	/*srv　播放哪个wav文件,音量有默认值*/
	ros::ServiceServer service_play_target_wav = n.advertiseService("play_taget_wav_srv", Play_Wav);

	/*srv 相对增大或减小音量，可以给定值，或者增加或减小原来的10%*/
	ros::ServiceServer service_adjust_voice = n.advertiseService("adujust_voice_srv", Adjust_Voice);

	/*srv 停止播放*/
	ros::ServiceServer service_stop_play = n.advertiseService("stop_to_play_srv", Stop_Play);
	ros::AsyncSpinner spinner(3);
	spinner.start();
	//ros::spin();
	ros::waitForShutdown();
	return 0;
}
