/*******************************************************
 HIDAPI - Multi-Platform library for
 communication with HID devices.
********************************************************/

#ifndef HIDAPI_H__
#define HIDAPI_H__

#include <wchar.h>
#include "protocol_proc_unit.h"

#ifdef _WIN32
#define HID_API_EXPORT __declspec(dllexport)
#define HID_API_CALL
#else
#define HID_API_EXPORT /**< API export macro */
#define HID_API_CALL   /**< API call macro */
#endif

#define HID_API_EXPORT_CALL HID_API_EXPORT HID_API_CALL /**< API export and call macro*/

#ifdef __cplusplus
extern "C"
{
#endif
/********************************************相关变量*******************************************/
	struct hid_device_;
	unsigned short VID;
	unsigned short PID;
	char *device_index = "iflytek";
	int is_reboot = 0;
	typedef struct hid_device_ hid_device; /**< opaque hidapi structure */
	/** hidapi info structure */
	struct hid_device_info
	{
		/** Platform-specific device path */
		char *path;
		/** Device Vendor ID */
		unsigned short vendor_id;
		/** Device Product ID */
		unsigned short product_id;
		/*The USB interface which this logical device*/
		int interface_number;
		/** Pointer to the next device */
		struct hid_device_info *next;
	};

/****************************************设备连接相关API**************************************/
	/*释放HIDAPI对象*/
	int HID_API_EXPORT HID_API_CALL hid_exit(void);
	/*打开麦克风设备*/
	HID_API_EXPORT hid_device *HID_API_CALL hid_open(void);
	/*关闭指定设备号的设备的连接*/
	void HID_API_EXPORT HID_API_CALL hid_close(void);
    //protocol_proc_init(send_to_usb_device, recv_from_usb_device, business_proc_callback, err_proc);

/****************************************设备启动相关API**************************************/
	/*获取系统状态，若开机则开始其他操作，若升级中，则下发资源信息及资源文件后再次开机*/
	void get_system_status();
	/*观察麦克风板是否升级成功*/
	void whether_upgrade_succeed(unsigned char *data);
	/*发送资源文件*/
	void send_resource(unsigned char *buf, char *fileName, int type);
	/*发送资源信息，即待发送文件的名字，md5,大小信息等*/
	void send_resource_info(char *fileName, int type);

/****************************************录音相关API******************************************/
	/*开始录制降噪音频*/
	void start_to_record_denoised_sound();
	/*停止录制降噪音频*/
	void finish_to_record_denoised_sound();
	/*开始录制原始音频*/
	void start_to_record_original_sound();
	/*停止录制原始音频*/
	void finish_to_record_original_sound();
	/*获取并保存原始音频文件到指定目录 */
	void get_original_sound(char *fileName_ori, unsigned char *data);
	/*获取并保存降噪后音频文件到指定目录 */
	void get_denoised_sound(char *fileName, unsigned char *data);
	/*设置主麦方向*/
	void set_major_mic_led_on(int id,int led_id);
    /*led 点亮规则--根据唤醒角度 */
	int get_led_based_angle(int mic_angle);
	/*led 点亮规则--根据设置的麦克风编号 */
	int get_led_based_mic_id(int mic_id);
    /*获取当前主麦方向 */
	void get_major_mic_id();
	/*获取当前被唤醒的麦克风方向*/
	int get_awake_mic_id(unsigned char *data, unsigned char *key);
	/*获取当前被唤醒的麦克风角度*/
	int get_awake_mic_angle(unsigned char* data,unsigned char* key);
	/*判断是否设置成功*/
	int whether_set_succeed(unsigned char *data, unsigned char *key);
	/*获取当前被唤醒的麦克风方向*/
	void whether_set_resource_info(unsigned char *data);
	int send_to_usb_device(const unsigned char *data, int size);
	int recv_from_usb_device(unsigned char *data, int size);
	int business_proc_callback(business_msg_t businessMsg);
	void err_proc(void);
    void recover_usb_connection(void);
	void sleep_ms(unsigned int secs);
#ifdef __cplusplus
}
#endif

#endif
