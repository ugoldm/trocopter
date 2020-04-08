import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

MIN_AREA = 400
MAX_AREA = 9000

GREEN_COLOUR = (45, 135, 95)
YELLOW_COLOUR = (215, 175, 40)
RED_COLOUR = (255, 75, 75)
WHITE_COLOUR = (255, 255, 255)
DELTA = 75
coords = [(0.295, 0.295), (0.885, 0.295), (0.295, 0.885), (0.885, 0.885),
         (0.295, 1.475), (0.885, 1.475), (0.295, 2.065), (0.885, 2.065), (0.59, 2.655)]


rospy.init_node('simple_recognition')
bridge = CvBridge()
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)

# Раскоментить для публикации
image_pub = rospy.Publisher('~debug', Image)


def image_callback(data):
    img = bridge.imgmsg_to_cv2(data, 'bgr8')

    # Много штук для улучшения качества распознавания формы
    imgGrey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, thrash = cv2.threshold(imgGrey, 240, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(
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
            main_colour = np.average(np.average(
                cropped_image, axis=0), axis=0)[::-1]

            if abs(main_colour - GREEN_COLOUR).sum() < DELTA:
                print("Зелёный")
            elif abs(main_colour - YELLOW_COLOUR).sum() < DELTA:
                print("Желтый")
            elif abs(main_colour - RED_COLOUR).sum() < DELTA:
                print("Красный")
            elif abs(main_colour - WHITE_COLOUR).sum() < DELTA:
                print("Белый")
    # Раскоментить для публикации
    image_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))

def navigate_wait(x, y, z, speed, frame_id, tolerance=0.1):
    navigate(x=x, y=y, z=z, speed=speed, frame_id=frame_id)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)


def main():
    image_sub = rospy.Subscriber(
        'main_camera/image_raw', Image, image_callback)

    navigate(x=0, y=0, z=1, speed=0.5, frame_id='body', auto_arm=True)
    rospy.sleep(4)
    #coords = [(0.295, 0.295), (0.885, 0.295), (0.295, 0.885), (0.885, 0.885),
    #     (0.295, 1.475), (0.885, 1.475), (0.295, 2.065), (0.885, 2.065), (0.59, 2.655)]
    for coord in coords:
        navigate_wait(x=coord[0], y=coord[1], z=1, speed=0.5, frame_id='map')
        rospy.sleep(2)
    navigate_wait(x=0, y=0, z=1, speed=0.5, frame_id='map')
    land()

    rospy.spin()


if __name__ == '__main__':
    main()