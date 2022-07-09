#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
import os
from apriltag_ros.srv import AnalyzeSingleImage, AnalyzeSingleImageRequest, AnalyzeSingleImageResponse
from sensor_msgs.msg import CameraInfo

DIR = os.path.dirname(os.path.abspath(__file__))
INPUT_FILE = os.path.join(DIR, "request.png")
RESULT_FILE = os.path.join(DIR, "response.png")

result_img = None


def imgCb(img_msg):
    img = cvbridge.imgmsg_to_cv2(img_msg, "bgr8")
    cv2.imwrite(INPUT_FILE, img, [cv2.IMWRITE_PNG_COMPRESSION, 0])
    request = AnalyzeSingleImageRequest(
        full_path_where_to_get_image=INPUT_FILE,
        full_path_where_to_save_image=RESULT_FILE,
        camera_info=cam_info,
    )
    response: AnalyzeSingleImageResponse = tag_service.call(request)
    print(response.tag_detections)
    global result_img
    result_img = cv2.imread(RESULT_FILE)
    cv2.imshow("result", result_img)


if __name__ == "__main__":
    rospy.init_node("test_tag_detect", anonymous=True)
    rospy.loginfo("等待apriltag服务")
    rospy.wait_for_service("/single_image_tag_detection")
    tag_service = rospy.ServiceProxy("/single_image_tag_detection", AnalyzeSingleImage)
    cvbridge = CvBridge()
    rospy.loginfo("等待camera_info...")
    cam_info: CameraInfo = rospy.wait_for_message("/usb_cam/camera_info", CameraInfo)
    while True:
        img_msg = rospy.wait_for_message("/usb_cam/image_rect_color", Image)
        imgCb(img_msg)
        if cv2.waitKey(0) == 27:
            break
