# encording=utf-8
# import sys, time, math
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

import numpy as np
import cv2  
import cv2.aruco as aruco
import math
import rospy
from geometry_msgs.msg import Twist

mtx = np.array([
        [ 420.11,       0,  319.60],
        [      0,  421.68,  244.82],
        [      0,       0,       1],
        ])
dist = np.array( [-0.318607, 0.097228, 0.000029, 0.000642, 0.] )

mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, mtx, (640, 480), 5)

id_to_find = 0
marer_size = 11

calib_path = ""
camera_matrix = mtx
camera_distortion = dist
directory = aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
parameters = aruco.DetectorParameters_create()

cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)
codec = cv2.VideoWriter.fourcc('M', 'J', 'P', 'G')
cap.set(cv2.CAP_PROP_FOURCC, codec)

def undistort(img):
    return cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)

def rotationMatrixToEulerAngles(rvecs):
    R = np.zeros((3, 3), dtype=np.float64)
    cv2.Rodrigues(rvecs, R)
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
    if  not singular:
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    #print('dst:', R)
    x = x*180.0/3.141592653589793
    y = y*180.0/3.141592653589793
    z = z*180.0/3.141592653589793
    return x,y,z

if __name__ == "__main__":
    rospy.init_node('qcode_detect', anonymous=True)
    detection_pub = rospy.Publisher("qcode_detect_result", Twist, queue_size=1)
    while True:
        ret, frame = cap.read()
        frame=undistort(frame)
        # frame=cv2.flip(frame,-1)
        frame = cv2.flip(frame,1)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        corners, ids, rejectedImgPoints = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters, cameraMatrix=camera_matrix,distCoeff=camera_distortion)  
        print("corners: ",corners)
    #    if ids != None and ids[0] == id_to_find:
        if type(ids) != type(None):
            ret = aruco.estimatePoseSingleMarkers(corners, marer_size, camera_matrix, camera_distortion)
            rvec, tvec = ret[0][0,0,:],ret[1][0,0,:]
            r,p,y = rotationMatrixToEulerAngles(rvec)
            print("r:%.2f,p:%.2f,y:%.2f"%(r,p,y))
            print("tvec: ",tvec)
            result = Twist()
            result.linear.x = tvec[0]
            result.linear.y = tvec[1]
            result.linear.z = tvec[2]
            result.angular.x = r
            result.angular.y = p
            result.angular.z = y
            detection_pub.publish(result)
            aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 50) #Draw Axis  
            aruco.drawDetectedMarkers(frame, corners) #Draw A square around the markers
        cv2.imshow("frame",frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            break
