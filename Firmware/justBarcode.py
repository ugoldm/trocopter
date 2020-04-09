import rospy
from pyzbar import pyzbar
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

bridge = CvBridge()

rospy.init_node('barcode_test')
image_pub = rospy.Publisher('~debug', Image, queue_size=1)
allowable = ("COVID - 19","healthy","Non COVID - 19")

def image_callback_qr(data):
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image
    barcodes = pyzbar.decode(cv_image)
    for barcode in barcodes:
        b_data = barcode.data.encode("utf-8")
        b_type = barcode.type
        (x, y, w, h) = barcode.rect
        xc = x + w/2
        yc = y + h/2
        if b_data in allowable:
	    cv2.putText(cv_image,b_data, (xc,yc))
	    print(b_data)
	image_pub.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8'))


image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback_qr, queue_size=1)
rospy.spin()
