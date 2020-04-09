#coding: utf8
import rospy
from clever import srv
from std_srvs.srv import Trigger
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import math
from pyzbar import pyzbar

MIN_AREA = 0
MAX_AREA = float("inf")

GREEN_COLOUR = (105, 165, 135)
YELLOW_COLOUR_1 = (145, 135, 88)
RED_COLOUR = (165, 100, 105)
WHITE_COLOUR = (255, 255, 255)

DELTA = 75

PATIENTS = [[0.295, 0.295], [0.295, 0.885], [0.295, 1.475],
            [0.295, 2.065], [0.59, 2.655], [0.885, 2.065],
            [0.885, 1.475], [0.885, 0.885], [0.885, 0.295]]

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)

allowable = ("COVID - 19", "healthy", "Non COVID - 19")
rospy.init_node('detection')
bridge = CvBridge()
image_pub = rospy.Publisher('~debug', Image, queue_size=1)
first_fly = True
check = 0
x = 180
y = 150
h = 100
w = 100


def image_callback(data):
    if first_fly and check == 1:
        img = bridge.imgmsg_to_cv2(data, 'bgr8')
        img_crop = img[150:500, 180:520]
        main_color = np.average(np.average(
            img_crop, axis=0), axis=0)[::-1]
        min_delta = min(abs(main_color - GREEN_COLOUR).sum(),
                        abs(main_color - YELLOW_COLOUR_1).sum(),
                        abs(main_color - RED_COLOUR).sum())
        if abs(main_color - GREEN_COLOUR).sum() < DELTA and abs(main_color - GREEN_COLOUR).sum() == min_delta:
            print("Зелёный")
            cv2.putText(img, "Зеленый", (x + w, y + h),
                        cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 3)

        elif abs(main_color - YELLOW_COLOUR_1).sum() < DELTA and abs(main_color - YELLOW_COLOUR_1).sum() == min_delta:
            print("Желтый")
            print("Сброшено")
            cv2.putText(img, "Желтый", (x + w, y + h),
                        cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 255), 3)

        elif abs(main_color - RED_COLOUR).sum() < DELTA and abs(main_color - RED_COLOUR).sum() == min_delta:
            print("Красный")
            print("Сброшено")
            cv2.putText(img, "Красный", (x + w, y + h),
                        cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 3)

        image_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))
    elif check == 1:
        cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image
        barcodes = pyzbar.decode(cv_image)
        for barcode in barcodes:
            b_data = barcode.data.encode("utf-8")
            b_type = barcode.type
            (x, y, w, h) = barcode.rect
            xc = x + w / 2
            yc = y + h / 2
            if b_data in allowable:
                cv2.putText(cv_image, b_data, (xc, yc),
                            cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0), 1)
                print(b_data)
        image_pub.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))


image_sub = rospy.Subscriber(
    'main_camera/image_raw_throttled', Image, image_callback)


navigate(x=0, y=0, z=0.6, speed=0.2, frame_id='body', auto_arm=True)
rospy.sleep(15)

navigate(x=0.295, y=0.295, z=0.6, speed=0.2, frame_id='aruco_map')
rospy.sleep(8)
check = 1
rospy.sleep(1)
check = 0


navigate(x=0.885, y=0.295, z=0.6, speed=0.2, frame_id='aruco_map')
rospy.sleep(8)
check = 1
rospy.sleep(1)
check = 0

navigate(x=0.295, y=0.885, z=0.6, speed=0.2, frame_id='aruco_map')
rospy.sleep(8)
check = 1
rospy.sleep(1)
check = 0

navigate(x=0.885, y=0.885, z=0.6, speed=0.2, frame_id='aruco_map')
rospy.sleep(8)
check = 1
rospy.sleep(1)
check = 0

navigate(x=0.295, y=1.475, z=0.6, speed=0.2, frame_id='aruco_map')
rospy.sleep(8)
check = 1
rospy.sleep(1)
check = 0

navigate(x=0.885, y=1.475, z=0.6, speed=0.2, frame_id='aruco_map')
rospy.sleep(8)
check = 1
rospy.sleep(1)
check = 0

navigate(x=0.295, y=2.065, z=0.6, speed=0.2, frame_id='aruco_map')
rospy.sleep(8)
check = 1
rospy.sleep(1)
check = 0

navigate(x=0.885, y=2.065, z=0.6, speed=0.2, frame_id='aruco_map')
rospy.sleep(8)
check = 1
rospy.sleep(1)
check = 0

navigate(x=0.59, y=2.655, z=0.6, speed=0.2, frame_id='aruco_map')
rospy.sleep(8)
check = 1
rospy.sleep(1)
check = 0

navigate(x=0, y=0, z=0.6, speed=0.2, frame_id='aruco_map')
rospy.sleep(8)

land()
rospy.sleep(60)
print("Wait 1 min")
first_fly = False
