import rospy
from clever import srv
from std_srvs.srv import Trigger

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)


navigate(x=0, y=0, z=1, speed=0.2, frame_id='body', auto_arm=True)
rospy.sleep(4)





navigate(x=0.295, y=0.295, z=0, speed=0.2, frame_id='body')
rospy.sleep(4)

navigate(x=0, y=0.59, z=0, speed=0.2, frame_id='body')
rospy.sleep(4)

navigate(x=0, y=0.59, z=0, speed=0.2, frame_id='body')
rospy.sleep(4)

navigate(x=0, y=0.59, z=0, speed=0.2, frame_id='body')
rospy.sleep(4)

navigate(x=0.295, y=0.59, z=0, speed=0.2, frame_id='body')
rospy.sleep(4)

navigate(x=0.295, y=-0.59, z=0, speed=0.2, frame_id='body')
rospy.sleep(4)

navigate(x=0, y=-0.59, z=0, speed=0.2, frame_id='body')
rospy.sleep(4)

navigate(x=0, y=-0.59, z=0, speed=0.2, frame_id='body')
rospy.sleep(4)

navigate(x=0, y=-0.59, z=0, speed=0.2, frame_id='body')
rospy.sleep(4)

land()
print("Succes! Congratulations!")