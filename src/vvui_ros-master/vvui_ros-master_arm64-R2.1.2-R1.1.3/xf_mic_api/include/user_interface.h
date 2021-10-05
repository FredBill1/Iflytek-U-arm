/* user_interface_h */
#include "asr_offline_record_sample.h"
/***************************参数配置区域，用户可通过修改这些词来进行 ********************************/
//录音文件保存的地址,最好设置为绝对地址
#define _GNU_SOURCE
#define ORIGINAL_SOUND_PATH "/audio/vvui_ori.pcm"
#define DENOISE_SOUND_PATH "/audio/vvui_deno.pcm"

//资源文件存储地址
#define SYSTEM_PATH "../tmp/system.tar"
#define SYSTEM_CONFIG_PATH "../tmp/config.txt"

//运行效果调试参数
int confidence = 40; //置信度阈值，可根据麦克风使用环境进行设置，若容易检测出错，则调大该值。
char awake_words[30] = "小Ｖ小Ｖ"; //使用的唤醒词
int time_per_order = 3;//一次命令时长
int awake_count = 5;//一次唤醒，可允许对话的次数
char *APPID = "5e159d11"; //APPID