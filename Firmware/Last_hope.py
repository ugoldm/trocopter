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

GREEN_COLOUR = (65, 115, 85)
YELLOW_COLOUR_1 = (175, 145, 58)
YELLOW_COLOUR_2 = (148, 127, 76)
YELLOW_COLOUR_3 = (173, 145, 62)
RED_COLOUR = (165, 70, 75)
WHITE_COLOUR = (255, 255, 255)
DELTA = 75

PATIENTS = [[0.295, 0.295], [0.295, 0.885], [0.295, 1.475],
			[0.295, 2.065], [0.59, 2.655], [0.885, 2.065],
            [0.885, 1.475], [0.885, 0.885], [0.885, 0.295]]

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)

rospy.init_node('detection')
bridge = CvBridge()
image_pub = rospy.Publisher('~debug', Image, queue_size=1)
first_fly = True


def is_near():
    map_now = get_telemetry(frame_id="map")
    tel_now = [map_now.x - map_begin.x, map_now.y - map_begin.y]
    if math.sqrt((tel_now[0] - PATIENTS[0][0])**2 + (tel_now[1] - PATIENTS[0][1])**2) < 0.2:
        return True
    else:
        return tel_now


def image_callback(data):

    if first_fly:
        img = bridge.imgmsg_to_cv2(data, 'bgr8')
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _, thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)
        _, contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL,
                                          cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > MIN_AREA and area < MAX_AREA:
                approx = cv2.approxPolyDP(
                    contour, 0.01 * cv2.arcLength(contour, True), True)

                cv2.drawContours(img, [approx], 0, (0, 0, 0), 5)
                x, y, w, h = cv2.boundingRect(contour)
                cropped_image = np.array(img[y:y + h, x:x + w])
                main_color = np.average(np.average(
                    cropped_image, axis=0), axis=0)[::-1]

                min_delta = min(abs(main_color - GREEN_COLOUR).sum(),
                                abs(main_color - YELLOW_COLOUR_1).sum(),
                                abs(main_color - YELLOW_COLOUR_2).sum(),
                                abs(main_color - YELLOW_COLOUR_3).sum(),
                                abs(main_color - RED_COLOUR).sum())

                if abs(main_color - GREEN_COLOUR).sum() < DELTA and abs(main_color - GREEN_COLOUR).sum() == min_delta:
                    print("Зелёный")
                    cv2.putText(img, "Зелёный", (x + w, y + h),
                                cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0), 1)

                elif abs(main_color - YELLOW_COLOUR_1).sum() < DELTA and abs(main_color - YELLOW_COLOUR_1).sum() == min_delta:
                    print("Желтый")
                    print("Сброшено")
                    cv2.putText(img, "Желтый", (x + w, y + h),
                                cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0), 1)
                elif abs(main_color - YELLOW_COLOUR_2).sum() < DELTA and abs(main_color - YELLOW_COLOUR_2).sum() == min_delta:
                    print("Желтый")
                    print("Сброшено")
                    cv2.putText(img, "Желтый", (x + w, y + h),
                                cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0), 1)
                elif abs(main_color - YELLOW_COLOUR_3).sum() < DELTA and abs(main_color - YELLOW_COLOUR_3).sum() == min_delta:
                    print("Желтый")
                    print("Сброшено")
                    cv2.putText(img, "Желтый", (x + w, y + h),
                                cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0), 1)

                elif abs(main_color - RED_COLOUR).sum() < DELTA and abs(main_color - RED_COLOUR).sum() == min_delta:
                    print("Красный")
                    print("Сброшено")
                    cv2.putText(img, "Красный", (x + w, y + h),
                                cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0), 1)

        # Раскоментить для публикации
        image_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))
    else:
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


map_begin = get_telemetry(frame_id="map")
print(map_begin.x)
print(map_begin.y)

navigate(x=0, y=0, z=0.7, speed=0.2, frame_id='body', auto_arm=True)
rospy.sleep(15)

navigate(x=0.295, y=0.295, z=0, speed=0.2, frame_id='body')
rospy.sleep(6)
print(is_near())
PATIENTS = PATIENTS[1:]

navigate(x=0, y=0.59, z=0, speed=0.2, frame_id='body')
rospy.sleep(6)
print(is_near())
PATIENTS = PATIENTS[1:]

navigate(x=0, y=0.59, z=0, speed=0.2, frame_id='body')
rospy.sleep(6)
print(is_near())
PATIENTS = PATIENTS[1:]

navigate(x=0, y=0.59, z=0, speed=0.2, frame_id='body')
rospy.sleep(6)
print(is_near())
PATIENTS = PATIENTS[1:]

navigate(x=0.295, y=0.59, z=0, speed=0.2, frame_id='body')
rospy.sleep(6)
print(is_near())
PATIENTS = PATIENTS[1:]

navigate(x=0.295, y=-0.59, z=0, speed=0.2, frame_id='body')
rospy.sleep(6)
print(is_near())
PATIENTS = PATIENTS[1:]

navigate(x=0, y=-0.59, z=0, speed=0.2, frame_id='body')
rospy.sleep(6)
print(is_near())
PATIENTS = PATIENTS[1:]

navigate(x=0, y=-0.59, z=0, speed=0.2, frame_id='body')
rospy.sleep(6)
print(is_near())
PATIENTS = PATIENTS[1:]

navigate(x=0, y=-0.59, z=0, speed=0.2, frame_id='body')
rospy.sleep(6)
print(is_near())

navigate(x=-0.885, y=-0.295, z=0, speed=0.2, frame_id='body')
rospy.sleep(8)

land()
rospy.sleep(60)
print("Wait 1 min")
first_fly = False

navigate(x=0, y=0, z=0.7, speed=0.2, frame_id='body', auto_arm=True)
rospy.sleep(15)

navigate(x=0.295, y=0.295, z=0, speed=0.2, frame_id='body')
rospy.sleep(6)
print(is_near())
PATIENTS = PATIENTS[1:]

navigate(x=0, y=0.59, z=0, speed=0.2, frame_id='body')
rospy.sleep(6)
print(is_near())
PATIENTS = PATIENTS[1:]

navigate(x=0, y=0.59, z=0, speed=0.2, frame_id='body')
rospy.sleep(6)
print(is_near())
PATIENTS = PATIENTS[1:]

navigate(x=0, y=0.59, z=0, speed=0.2, frame_id='body')
rospy.sleep(6)
print(is_near())
PATIENTS = PATIENTS[1:]

navigate(x=0.295, y=0.59, z=0, speed=0.2, frame_id='body')
rospy.sleep(6)
print(is_near())
PATIENTS = PATIENTS[1:]

navigate(x=0.295, y=-0.59, z=0, speed=0.2, frame_id='body')
rospy.sleep(6)
print(is_near())
PATIENTS = PATIENTS[1:]

navigate(x=0, y=-0.59, z=0, speed=0.2, frame_id='body')
rospy.sleep(6)
print(is_near())
PATIENTS = PATIENTS[1:]

navigate(x=0, y=-0.59, z=0, speed=0.2, frame_id='body')
rospy.sleep(6)
print(is_near())
PATIENTS = PATIENTS[1:]

navigate(x=0, y=-0.59, z=0, speed=0.2, frame_id='body')
rospy.sleep(6)
print(is_near())

navigate(x=-0.885, y=-0.295, z=0, speed=0.2, frame_id='body')
rospy.sleep(8)
land()
