#coding: utf8
import rospy
from clever import srv
from std_srvs.srv import Trigger
import math


coords = [(0.295, 0.295), (0.885, 0.295), (0.295, 0.885), (0.885, 0.885),
          (0.295, 1.475), (0.885, 1.475), (0.295, 2.065), (0.885, 2.065), (0.59, 2.655)]
rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)


def navigate_wait(x, y, z, speed, frame_id, tolerance=0.1):
    navigate(x=x, y=y, z=z, speed=speed, frame_id=frame_id)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)


def main():
    navigate(x=0, y=0, z=1, speed=0.5, frame_id='body', auto_arm=True)
    rospy.sleep(4)
    for coord in coords:
        navigate_wait(x=coord[0], y=coord[1], z=1, speed=0.5, frame_id='map')
        rospy.sleep(2)
    navigate_wait(x=0, y=0, z=1, speed=0.5, frame_id='map')
    land()


if __name__ == '__main__':
    main()