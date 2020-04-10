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
from clever.srv import SetLEDEffect

MIN_AREA = 0
MAX_AREA = float("inf")

GREEN_COLOUR = (125, 135, 135)
YELLOW_COLOUR_1 = (145, 135, 97)

RED_COLOUR = (155, 117, 125)

DELTA = 75

PATIENTS = {'1': [0.295, 0.295], '2': [0.885, 0.295], '3': [0.295, 0.885],
            '4': [0.885, 0.885], '5': [0.295, 1.475], '6': [0.885, 1.475],
            '7': [0.295, 2.065], '8': [0.885, 2.065], '9': [0.59, 2.655]}
SUSPECTS = {}

set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)

allowable = ("COVID - 19", "healthy", "Non COVID - 19")
rospy.init_node('detection')
bridge = CvBridge()
image_pub = rospy.Publisher('~debug', Image, queue_size=1)
first_fly = True
check = '0'
x = 180
y = 150
h = 100
w = 100
arr = []

#Функция для определения самого частого элемента в массиве
def most_frequent(List):
    try:
        return max(set(List), key=List.count)
    except:
        return "green_"

#Основной колл-бек, включающий в себя распознавание цвета и QR кодов.
def image_callback(data):
    if first_fly and check != '0':
        global x
        global y
        global h
        global w
        global arr
        #Получаем изображения с камеры
        img = bridge.imgmsg_to_cv2(data, 'bgr8')
        #Обрезаем по центру
        img_crop = img[150:500, 180:520]
        #Определяем основной цвет
        main_color = np.average(np.average(
            img_crop, axis=0), axis=0)[::-1]
        #Считаем наименьшее отклонения от констант
        min_delta = min(abs(main_color - GREEN_COLOUR).sum(),
                        abs(main_color - YELLOW_COLOUR_1).sum(),
                        abs(main_color - RED_COLOUR).sum())
        #Проверяем подходит ли отклонение под DELTA и сравниваем с минимальным
        if abs(main_color - GREEN_COLOUR).sum() < DELTA and abs(main_color - GREEN_COLOUR).sum() == min_delta:
            #print(check + ": Зелёный")
            #Помещаем текст на фото
            cv2.putText(img_crop, "Зеленый", (x + w, y + h),
                        cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 3)
            #Добавляем в массив цвет
            arr.append("green")

        elif abs(main_color - YELLOW_COLOUR_1).sum() < DELTA and abs(main_color - YELLOW_COLOUR_1).sum() == min_delta:
            #print(check + ": Желтый")
            #print("Сброшено")
            cv2.putText(img_crop, "Желтый", (x + w, y + h),
                        cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 255), 3)
            arr.append("yellow")

        elif abs(main_color - RED_COLOUR).sum() < DELTA and abs(main_color - RED_COLOUR).sum() == min_delta:
            #print(check + ": Красный")
            #print("Сброшено")
            cv2.putText(img_crop, "Красный", (x + w, y + h),
                        cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 3)
            arr.append("red")

        image_pub.publish(bridge.cv2_to_imgmsg(img_crop, 'bgr8'))
    elif check != '0':
        #Получаем изображения с камеры
        cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image
        #Получаем все QR коды
        barcodes = pyzbar.decode(cv_image)
        for barcode in barcodes:
            #расшифровываем QR
            b_data = barcode.data.encode("utf-8")
            b_type = barcode.type
            (x, y, w, h) = barcode.rect
            xc = x + w / 2
            yc = y + h / 2
            #Если это один из допустимых QR,то добавляем вывод(чтобы не путуть с ArUco)
            if b_data in allowable:
                cv2.putText(cv_image, b_data, (xc, yc),
                            cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 1)
                print(check+": "+b_data)
                arr.append(b_data)
        #Публикуем изображение в топик
        image_pub.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))

#Оформляем подписку на топик камеры с колл-беком
image_sub = rospy.Subscriber(
    'main_camera/image_raw_throttled', Image, image_callback)

#Взлет
navigate(x=0, y=0, z=0.65, speed=0.2, frame_id='body', auto_arm=True)
rospy.sleep(15)

#Летим в первую точку
navigate(x=0.295, y=0.295, z=0.65, speed=0.2, frame_id='aruco_map')
rospy.sleep(8)
#Запускаем распознование
check = '1'
rospy.sleep(1)
#Останавливаем распознование
check = '0'
#Если цвет красный или желтый, добавляем эту точку в словарь для второго облета

print(most_frequent(arr))
if most_frequent(arr) in ("red", "yellow"):
    SUSPECTS["1"] = most_frequent(arr)
    #Делаем сигнал светодиодной лентой
    set_effect(effect='blink', r=148, g=0, b=211)
    rospy.sleep(5)
    set_effect(effect='fill', r=255, g=255, b=255)
    print("Сброшено")
#Очищаем массив
arr = []

navigate(x=0.885, y=0.295, z=0.65, speed=0.2, frame_id='aruco_map')
rospy.sleep(8)
check = '2'
rospy.sleep(1)
check = '0'
print(most_frequent(arr))
if most_frequent(arr) in ("red", "yellow"):
    SUSPECTS["2"] = most_frequent(arr)
    set_effect(effect='blink', r=148, g=0, b=211)
    rospy.sleep(5)
    set_effect(effect='fill', r=255, g=255, b=255)
    print("Сброшено")

arr = []

navigate(x=0.295, y=0.885, z=0.65, speed=0.2, frame_id='aruco_map')
rospy.sleep(8)
check = '3'
rospy.sleep(1)
check = '0'
print(most_frequent(arr))
if most_frequent(arr) in ("red", "yellow"):
    SUSPECTS["3"] = most_frequent(arr)
    set_effect(effect='blink', r=148, g=0, b=211)
    rospy.sleep(5)
    set_effect(effect='fill', r=255, g=255, b=255)
    print("Сброшено")

arr = []

navigate(x=0.885, y=0.885, z=0.65, speed=0.2, frame_id='aruco_map')
rospy.sleep(8)
check = '4'
rospy.sleep(1)
check = '0'
print(most_frequent(arr))
if most_frequent(arr) in ("red", "yellow"):
    SUSPECTS["4"] = most_frequent(arr)
    set_effect(effect='blink', r=148, g=0, b=211)
    rospy.sleep(5)
    set_effect(effect='fill', r=255, g=255, b=255)
    print("Сброшено")

arr = []

navigate(x=0.295, y=1.475, z=0.65, speed=0.2, frame_id='aruco_map')
rospy.sleep(8)
check = '5'
rospy.sleep(1)
check = '0'
print(most_frequent(arr))
if most_frequent(arr) in ("red", "yellow"):
    SUSPECTS["5"] = most_frequent(arr)
    set_effect(effect='blink', r=148, g=0, b=211)
    rospy.sleep(5)
    set_effect(effect='fill', r=255, g=255, b=255)
    print("Сброшено")

arr = []

navigate(x=0.885, y=1.475, z=0.65, speed=0.2, frame_id='aruco_map')
rospy.sleep(8)
check = '6'
rospy.sleep(1)
check = '0'
print(most_frequent(arr))
if most_frequent(arr) in ("red", "yellow"):
    SUSPECTS["6"] = most_frequent(arr)
    set_effect(effect='blink', r=148, g=0, b=211)
    rospy.sleep(5)
    set_effect(effect='fill', r=255, g=255, b=255)
    print("Сброшено")

arr = []

navigate(x=0.295, y=2.065, z=0.65, speed=0.2, frame_id='aruco_map')
rospy.sleep(8)
check = '7'
rospy.sleep(1)
check = '0'
print(most_frequent(arr))
if most_frequent(arr) in ("red", "yellow"):
    SUSPECTS["7"] = most_frequent(arr)
    set_effect(effect='blink', r=148, g=0, b=211)
    rospy.sleep(5)
    set_effect(effect='fill', r=255, g=255, b=255)
    print("Сброшено")

arr = []

navigate(x=0.885, y=2.065, z=0.65, speed=0.2, frame_id='aruco_map')
rospy.sleep(8)
check = '8'
rospy.sleep(1)
check = '0'
print(most_frequent(arr))
if most_frequent(arr) in ("red", "yellow"):
    SUSPECTS["8"] = most_frequent(arr)
    set_effect(effect='blink', r=148, g=0, b=211)
    rospy.sleep(5)
    set_effect(effect='fill', r=255, g=255, b=255)
    print("Сброшено")

arr = []

navigate(x=0.59, y=2.655, z=0.65, speed=0.2, frame_id='aruco_map')
rospy.sleep(8)
check = '9'
rospy.sleep(1)
check = '0'
print(most_frequent(arr))
if most_frequent(arr) in ("red", "yellow"):
    SUSPECTS["9"] = most_frequent(arr)
    set_effect(effect='blink', r=148, g=0, b=211)
    rospy.sleep(5)
    set_effect(effect='fill', r=255, g=255, b=255)
    print("Сброшено")
arr = []

navigate(x=0, y=0, z=0.65, speed=0.2, frame_id='aruco_map')
rospy.sleep(18)

land()
print("Wait 1 min")
rospy.sleep(60)
first_fly = False

navigate(x=0, y=0, z=0.65, speed=0.2, frame_id='body', auto_arm=True)
rospy.sleep(8)

for i in list(SUSPECTS.keys()):
    coord = PATIENTS[i]
    navigate(x=coord[0], y=coord[1], z=0.65, speed=0.2, frame_id='aruco_map')
    rospy.sleep(8)
    check = i
    rospy.sleep(1)
    check = "0"
    if most_frequent(arr) == "COVID - 19":
        set_effect(effect='blink', r=255, g=0, b=0)
        rospy.sleep(5)
        set_effect(effect='fill', r=255, g=255, b=255)       
    arr = []


navigate(x=0, y=0, z=0.65, speed=0.2, frame_id='aruco_map')
rospy.sleep(10)
land()
print("Yeah!")
