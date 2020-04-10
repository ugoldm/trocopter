# trocopter

Данный репозиторий содержит описание работы кода команды Трокоптер.
Алгоритм мы разделили на несколько частей: первый облет и второй облет.

Во время первого облета требовалось:
1) Пролететь по заданным координатам.
2) Распознать цвет метки на координате.
3) Просигнализировать светодиодной лентой, если цвет желтый или красный.
4) "Запомнить", где цвет желтый или красный, чтобы во время второго облета вернуться на эту точку.

<<<<<<< HEAD
�� ����� ������� ������ �����������:
1) ��������� ����� �� "�����������" �����������
2) �������������� QR ����� �� ��� �����������
3) ������������������ ������, ���� ��������� �������������, � ������� ��������� � �������� 

������ � ������� ��������.
1) ��� ������ �� ����������� �� ���������� ������� navigate() � frame_id = 'aruco_map'
'''python
navigate(x=0.295, y=0.885, z=0.5, speed=0.2, frame_id='aruco_map')
rospy.sleep(8)
'''
����� ������� ������ ��������� �� ���� �����������.

2) ������������ ���� �� �������� ����� ������ ��������� � ��������� �����. ��� ����, ����� � ���������� ������ ������������ ��, ��� ��� ����, �� ���������� ��������� �����������
'''python
if first_fly and check != '0':
'''
���������� first_fly ����� True �� ����� ������� ������, � False �� ����� �������. ����� �������, �� ����� ������� ������ �� ���������� �����, � �� ����� ������� QR ����, ��������� ���� callback.
���������� check ��������� ��������, ��������������� ������ �����
'''python
navigate(x=0.295, y=0.295, z=0.5, speed=0.2, frame_id='aruco_map')
rospy.sleep(8)
#��������� �������������
check = '1'
rospy.sleep(1)
#������������� �������������
check = '0'
'''
������ ����� ����������� � ������������ � ������� PATIENTS
'''python
PATIENTS = {'1': [0.295, 0.295], '2': [0.885, 0.295], '3': [0.295, 0.885],
            '4': [0.885, 0.885], '5': [0.295, 1.475], '6': [0.885, 1.475],
            '7': [0.295, 2.065], '8': [0.885, 2.065], '9': [0.59, 2.655]}
'''
����� �������, ����� ������ ��������� �� �����-���� ����������, ������������� �������� ��������.
=======
Во время второго облета требовалось:
1) Совершить облет по "запомненным" координатам.
2) Просканировать QR метки на эих координатах.
>>>>>>> 6bf30a00265798262e4205033d2e0aec1a6cf3ae
