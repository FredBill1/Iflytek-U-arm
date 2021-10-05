#ifndef __USERDEF_H
#define __USERDEF_H

//0： 关闭log打印  
//1： C log  
//2： android log
#define debug_log 1
#define LOGCAT_DEBUG 1						
// log标签
#if (LOGCAT_DEBUG == 2)
	#include <android/log.h>
	#define  CUSTOM_TAG "yjzxService"
	// 定义info信息
	#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, CUSTOM_TAG, __VA_ARGS__)
	// 定义debug信息
	#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, CUSTOM_TAG, __VA_ARGS__)
	// 定义warn信息
	#define LOGW(...) __android_log_print(ANDROID_LOG_WARN, CUSTOM_TAG, __VA_ARGS__)
	// 定义error信息
	#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, CUSTOM_TAG, __VA_ARGS__)
#elif (LOGCAT_DEBUG == 1)
	// 定义info信息
	#define LOGI(format, ...) printf("[INFO] " format";  Line:%d, Function:%s, File:%s\n", ##__VA_ARGS__, __LINE__, __FUNCTION__, __FILE__);
	// 定义debug信息
	#define LOGD(format, ...) printf("[DEBUG] " format";  Line:%d, Function:%s, File:%s\n", ##__VA_ARGS__, __LINE__, __FUNCTION__, __FILE__);
	// 定义warn信息
	#define LOGW(format, ...) printf("[WARN] " format";  Line:%d, Function:%s, File:%s\n", ##__VA_ARGS__, __LINE__, __FUNCTION__, __FILE__);
	// 定义error信息
	#define LOGE(format, ...) printf("[ERROR] " format";  Line:%d, Function:%s, File:%s\n", ##__VA_ARGS__, __LINE__, __FUNCTION__, __FILE__);
#else
	#define LOGI(format, ...) printf("[INFO] " format";  Line:%d, Function:%s, File:%s\n", ##__VA_ARGS__, __LINE__, __FUNCTION__, __FILE__);
	// 定义debug信息
	#define LOGD(format, ...)
	// 定义warn信息
	#define LOGW(format, ...)
	// 定义error信息
	#define LOGE(format, ...)
#endif

 #endif
