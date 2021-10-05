/****************************************************************************/
/*iflytek_tts:
subscrible:"/vvui/text_out" - the text accepted from the  node of iflytek_nlu  
/****************************************************************************/

#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <string>
#include "qtts.h"
#include "msp_cmn.h"
#include "msp_errors.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <xf_mic_tts_online/Play_TTS_Online_srv.h>

using namespace std;

string pkg_path = " ";
string tts_topic = "/xiao/mic/tts";         //default tts topic
static std::string appID = "5d1c5791";      //可更换为自己的APPID
static std::string speakerName = "xiaoyan"; //由于是在线语音识别，可以自己选择多种播报人声音
static int voice_speed = 50;                //播放的音量，可与系统音量结合进行播放声音的增大或缩小
string tts_service = "play_txt_wav";        //default tts topic

/* wav音频头部格式 */
typedef struct _wave_pcm_hdr
{
    char riff[4]; // = "RIFF"
    int size_8;   // = FileSize - 8
    char wave[4]; // = "WAVE"
    char fmt[4];  // = "fmt "
    int fmt_size; // = 下一个结构体的大小 : 16

    short int format_tag;      // = PCM : 1
    short int channels;        // = 通道数 : 1
    int samples_per_sec;       // = 采样率 : 8000 | 6000 | 11025 | 16000
    int avg_bytes_per_sec;     // = 每秒字节数:samples_per_sec*bits_per_sample/8
    short int block_align;     // = 每采样点字节数 : wBitsPerSample / 8
    short int bits_per_sample; // = 量化比特数: 8 | 16

    char data[4];  // = "data";
    int data_size; // = 纯数据长度 : FileSize - 44
} wave_pcm_hdr;

/* 默认wav音频头部数据 */
wave_pcm_hdr default_wav_hdr =
    {
        {'R', 'I', 'F', 'F'},
        0,
        {'W', 'A', 'V', 'E'},
        {'f', 'm', 't', ' '},
        16,

        1,
        1,
        16000,
        32000,
        2,
        16,

        {'d', 'a', 't', 'a'},
        0};

/* 文本合成，即音频文件的写入过程 */
int text_to_speech(const char *src_text, const char *des_path, const char *params)
{
    int ret = -1;
    FILE *fp = NULL;
    const char *sessionID = NULL;
    unsigned int audio_len = 0;
    wave_pcm_hdr wav_hdr = default_wav_hdr;
    int synth_status = MSP_TTS_FLAG_STILL_HAVE_DATA;

    if (NULL == src_text || NULL == des_path)
    {
        printf("params is error!\n");
        return ret;
    }

    fp = fopen(des_path, "wb");
    if (NULL == fp)
    {
        printf("open ( %s ) error.\n", des_path);
        return ret;
    }

    /* 开始合成 */
    sessionID = QTTSSessionBegin(params, &ret);
    if (MSP_SUCCESS != ret)
    {
        printf("QTTSSessionBegin failed, error code: %d.\n", ret);
        fclose(fp);
        return ret;
    }
    ret = QTTSTextPut(sessionID, src_text, (unsigned int)strlen(src_text), NULL);
    if (MSP_SUCCESS != ret)
    {
        printf("QTTSTextPut failed, error code: %d.\n", ret);
        QTTSSessionEnd(sessionID, "TextPutError");
        fclose(fp);
        return ret;
    }
    fwrite(&wav_hdr, sizeof(wav_hdr), 1, fp); //添加wav音频头，使用采样率为16000
    while (1)
    {
        /* 获取合成音频 */
        const void *data = QTTSAudioGet(sessionID, &audio_len, &synth_status, &ret);
        if (MSP_SUCCESS != ret)
            break;

        if (NULL != data)
        {
            fwrite(data, audio_len, 1, fp);
            wav_hdr.data_size += audio_len; //计算data_size大小
        }
        if (MSP_TTS_FLAG_DATA_END == synth_status)
            break;
        printf(">");
        usleep(30 * 1000); //防止频繁占用CPU
    }                      //合成状态synth_status取值请参阅《讯飞语音云API文档》
    printf("\n");

    if (MSP_SUCCESS != ret)
    {
        printf("QTTSAudioGet failed, error code: %d.\n", ret);
        QTTSSessionEnd(sessionID, "AudioGetError");
        fclose(fp);
        return ret;
    }

    /* 修正wav文件头数据的大小 */
    wav_hdr.size_8 += wav_hdr.data_size + (sizeof(wav_hdr) - 8);

    /* 将修正过的数据写回文件头部,音频文件为wav格式 */
    fseek(fp, 4, 0);
    fwrite(&wav_hdr.size_8, sizeof(wav_hdr.size_8), 1, fp);       //写入size_8的值
    fseek(fp, 40, 0);                                             //将文件指针偏移到存储data_size值的位置
    fwrite(&wav_hdr.data_size, sizeof(wav_hdr.data_size), 1, fp); //写入data_size的值
    fclose(fp);
    fp = NULL;

    /* 合成完毕 */
    ret = QTTSSessionEnd(sessionID, "Normal");
    if (MSP_SUCCESS != ret)
    {
        printf("QTTSSessionEnd failed, error code: %d.\n", ret);
    }

    return ret;
}

//调用在线的语音合成引擎进行音频文件的合成，需要登录信息以及资源信息
int xf_mic_tts(const char *text, const char *filename, const char *pAppID, const char *pSpeaker)
{
    int ret = MSP_SUCCESS;
    char login_params[35];
    char session_begin_params[120];

    memset(login_params, 0, sizeof(login_params));
    memset(session_begin_params, 0, sizeof(session_begin_params));
    //const char* login_params         = "appid = 5d1c5791, work_dir = .";//登录参数,appid与msc库绑定,请勿随意改动
    sprintf(login_params, "appid = %s, work_dir = .", pAppID);
    sprintf(session_begin_params, "voice_name = %s, text_encoding = utf8, sample_rate = 16000, speed = %d, volume = 80, pitch = 50, rdn = 0", pSpeaker, voice_speed);

    printf("login_params: %s\n", login_params);
    printf("session_begin_params:%s\n", session_begin_params);

    /* 用户登录 */
    ret = MSPLogin(NULL, NULL, login_params); //第一个参数是用户名，第二个参数是密码，第三个参数是登录参数，用户名和密码可在http://open.voicecloud.cn注册获取
    if (MSP_SUCCESS != ret)
    {
        printf("MSPLogin failed, error code: %d.\n", ret);
        MSPLogout();
        return 1;
    }

    /* 文本合成 */
    printf("语音开始合成 ...\n");
    ret = text_to_speech(text, filename, session_begin_params);
    if (MSP_SUCCESS != ret)
    {
        printf("text_to_speech failed, error code: %d.\n", ret);
        return 2;
    }
    printf("合成完毕\n");

exit:
    MSPLogout(); //退出登录
    return 0;
}

/**
 * 播放音频文件，音频文件是保存在本地的，可调用system的play进行播放
 */
void playVoice()
{
    string play_audio_path = "play " + pkg_path + "/audio/say.wav";
    system(play_audio_path.c_str());
}

/**
 * 回调函数，接受的话题内容为要播报的内容
 */
bool xf_mic_tts_callback(xf_mic_tts_online::Play_TTS_Online_srv::Request &req,
                         xf_mic_tts_online::Play_TTS_Online_srv::Response &res)
{
    char cmd[2000];
    if (req.text.size() > 1)
    {
        std::cout << "I will speak:" << req.text.c_str() << std::endl;
        //语音合成到本地路径
        string path = pkg_path + "/audio/say.wav";
        if (req.appID.size() > 2)
        {
            appID = req.appID;
        }
        if (req.speakerName.size() > 2)
        {
            if (req.speakerName == "xiaoyan" || req.speakerName == "aisjiuxu" || req.speakerName == "aisxping" || req.speakerName == "aisxping" || req.speakerName == "aisbabyxu")
            {
                speakerName = req.speakerName;
            }
            else
            {
                res.result = "fail";
                res.fail_reason = "speaker_name_error";
            }
        }
        int ret = xf_mic_tts(req.text.c_str(), path.c_str(), appID.c_str(),speakerName.c_str());
        if (ret == 1)
        {
            res.result = "fail";
            res.fail_reason = "login_error";
        }
        else if (ret == 2)
        {
            res.result = "fail";
            res.fail_reason = "unfit_appid_and_lib_error";
        }
        else
        {
            playVoice();
            res.result = "ok";
            res.fail_reason = "";
        }
    }
    else
    {
        res.result = "fail";
        res.fail_reason = "text is null_error";
    }
    ROS_INFO("send result to client,finish to paly text\n");
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "xf_tts_online_node");
    ros::NodeHandle n("~");

    n.param("source_path", pkg_path, std::string("./"));
    n.param("app_id", appID, std::string("5d1c5791"));
    ros::param::get("speaker_name", speakerName);
    ros::param::get("~tts_service", tts_service);
    ros::param::get("voice_speed", voice_speed);

    //订阅函数声明
    ros::ServiceServer service_play_tts = n.advertiseService(tts_service, xf_mic_tts_callback);
    ros::spin();

    return 0;
}
