import rospy
from pyzbar import pyzbar
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from clever import srv
from std_srvs.srv import Trigger

bridge = CvBridge()
rospy.init_node('barcode_test')

#image_pub = rospy.Publisher('~debug', Image)
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)
# Image subscriber callback function
def image_callback_qr(data):
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image
    barcodes = pyzbar.decode(cv_image)
    for barcode in barcodes:
        b_data = barcode.data.encode("utf-8")
        b_type = barcode.type
        (x, y, w, h) = barcode.rect
        xc = x + w/2
        yc = y + h/2
        print ("Found {} with data {} with center at x={}, y={}".format(b_type, b_data, xc, yc))


def main():
	image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback_qr, queue_size=1)
	navigate(x=0, y=0, z=1, speed=0.3, frame_id='body', auto_arm=True)
    rospy.sleep(10)
    land()
	rospy.spin()

    if __name__ == '__main__':
    main()