#coding: utf8
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

MIN_AREA = 400
MAX_AREA = float("inf")

GREEN_COLOUR = (55, 95, 75)
YELLOW_COLOUR = (175, 145, 50)
RED_COLOUR = (165, 70, 75)
WHITE_COLOUR = (255, 255, 255)
DELTA = 75

rospy.init_node('simple_recognition')
bridge = CvBridge()
# Раскоментить для публикации
image_pub = rospy.Publisher('~debug', Image,queue_size=None)


def image_callback(data):
    img = bridge.imgmsg_to_cv2(data, 'bgr8')

    # Много штук для улучшения качества распознавания формы
    imgGrey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, thrash = cv2.threshold(imgGrey, 240, 255, cv2.THRESH_BINARY)
    _, contours, _ = cv2.findContours(
        thrash, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    for contour in contours:
        area = cv2.contourArea(contour)
        if area > MIN_AREA and area < MAX_AREA:
            approx = cv2.approxPolyDP(
                contour, 0.01 * cv2.arcLength(contour, True), True)
            # Если нужна надпись
            #x_title = approx.ravel()[0]
            #y_title = approx.ravel()[1]

            cv2.drawContours(img, [approx], 0, (0, 0, 0), 5)

            x, y, w, h = cv2.boundingRect(contour)
            cropped_image = np.array(img[y:y + h, x:x + w])
            main_color = np.average(np.average(
                cropped_image, axis=0), axis=0)[::-1]

            if abs(main_color - GREEN_COLOUR).sum() < DELTA:
                print("Зелёный")
            elif abs(main_color - YELLOW_COLOUR).sum() < DELTA:
                print("Желтый")
            elif abs(main_color - RED_COLOUR).sum() < DELTA:
                print("Красный")
            elif abs(main_color - WHITE_COLOUR).sum() < DELTA:
                print("Белый")
    # Раскоментить для публикации
    image_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))


def main():
    image_sub = rospy.Subscriber(
        'main_camera/image_raw', Image, image_callback)

    rospy.spin()


if __name__ == '__main__':
    main()
