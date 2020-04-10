# trocopter

Данный репозиторий содержит описание работы кода команды Трокоптер.
Алгоритм мы разделили на несколько частей: первый облет и второй облет.

Во время первого облета требовалось:
1) Пролететь по заданным координатам
2) Распознать цвет метки на координате
3) Просигнализировать светодиодной лентой, если цвет желтый или красный
4) "Запомнить", где цвет желтый или красный, чтобы во время второго облета вернуться на эту точку.

Во время второго облета требовалось:
1) Совершить облет по "запомненным" координатам
2) Просканировать QR метки на эих координатах
3) Просигнализировать лентой, если заражение подтвердилось, и вывести результат в терминал 

Начнем с первого облетета.
1) Для полета по координатам мы используем функцию navigate() с frame_id = 'aruco_map'
```python
navigate(x=0.295, y=0.885, z=0.5, speed=0.2, frame_id='aruco_map')
rospy.sleep(8)
```
Таким образом коптер пролетает по всем координатам.

2) Распознавать цвет мы начинаем когда коптер прилетает в очередную точку. Для того, чтобы в конкретный момент распознавать то, что нам надо, мы используем следующую конструкцию
```python
if first_fly and check != '0':
```
Переменная first_fly равна True во время первого облета, и False во время второго. Таким образом, во время первого полета мы распознаем цвета, а во время второго QR коды, используя один callback.
Переменная check принимает значение, соответствующее номеру точки
```python
navigate(x=0.295, y=0.295, z=0.5, speed=0.2, frame_id='aruco_map')
rospy.sleep(8)
#Запускаем распознование
check = '1'
rospy.sleep(1)
#Останавливаем распознование
check = '0'
```
Номера точек соотносятся с координатами в словаре PATIENTS
```python
PATIENTS = {'1': [0.295, 0.295], '2': [0.885, 0.295], '3': [0.295, 0.885],
            '4': [0.885, 0.885], '5': [0.295, 1.475], '6': [0.885, 1.475],
            '7': [0.295, 2.065], '8': [0.885, 2.065], '9': [0.59, 2.655]}
```
Таким образом, когда коптер находится на какой-либо координате, распознавалка начинает работать.

Алгоритм распознавания работает следующим образом:
Мы берем изображение с камеры, обрезаем его по центру, находим доминантный цвет и сравниваем его с заданными значениями
```python
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
            print(check + ": Зелёный")
            #Помещаем текст на фото
            cv2.putText(img, "Зеленый", (x + w, y + h),
                        cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 3)
            #Добавляем в массив цвет
            arr.append("green")
```
Поскольку распознавалка успевает сработать несколько раз, у нее образуется несколько выводов. Все выводы записываются в один массив
```python
arr.append("green")
```
Затем мы определяем, какое из значений встречается чаще всего
```python
def most_frequent(List):
    try:
        return max(set(List), key=List.count)
    except:
        return " none"
```
