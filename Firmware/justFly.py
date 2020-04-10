import rospy
from clever import srv
from std_srvs.srv import Trigger

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)

navigate(x=0, y=0, z=0.6, speed=0.2, frame_id='body', auto_arm=True)
rospy.sleep(3) #15
print("navigate done 1 out of 9")

navigate(x=0.295, y=0.295, z=0.5, speed=0.2, frame_id='aruco_map')
rospy.sleep(3) #8
print("navigate done 2 out of 9")

navigate(x=0.885, y=0.295, z=0.5, speed=0.2, frame_id='aruco_map')
rospy.sleep(3)
print("navigate done 3 out of 9")

navigate(x=0.295, y=0.885, z=0.5, speed=0.2, frame_id='aruco_map')
rospy.sleep(3)
print("navigate done 4 out of 9")

navigate(x=0.885, y=0.885, z=0.5, speed=0.2, frame_id='aruco_map')
rospy.sleep(3)
print("navigate done 5 out of 9")

navigate(x=0.295, y=1.475, z=0.5, speed=0.2, frame_id='aruco_map')
rospy.sleep(3)
print("navigate done 6 out of 9")

navigate(x=0.885, y=1.475, z=0.5, speed=0.2, frame_id='aruco_map')
rospy.sleep(3)
print("navigate done 7 out of 9")

navigate(x=0.295, y=2.065, z=0.5, speed=0.2, frame_id='aruco_map')
rospy.sleep(3)
print("navigate done 8 out of 9")

navigate(x=0.885, y=2.065, z=0.5, speed=0.2, frame_id='aruco_map')
rospy.sleep(3)
print("navigate done 9 out of 9")

navigate(x=0.59, y=2.655, z=0.5, speed=0.2, frame_id='aruco_map')
rospy.sleep(3)
print("navigate done 10 out of 9")

navigate(x=0, y=0, z=0.5, speed=0.2, frame_id='aruco_map')
rospy.sleep(3)
print("navigate done 11 out of 9")
land()