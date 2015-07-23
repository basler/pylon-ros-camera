#! /usr/bin/python2
from django.template.defaultfilters import last

import cv2
from pylon_camera_msgs.srv import SetBrightnessSrv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rospy
from std_msgs.msg import Float32


def get_brightness(img, full_image):


    #print "accepted"
    if full_image:
        # print 'full'
        return cv2.mean(img)[0]

    else:
        #print img.shape
        s = img.shape
        w = s[0]
        h = s[1]
        return cv2.mean(img[0.25*w:0.75*w,0.25*h:0.75*h])[0]



def img_cb(msg):
    global pub
    cv_image = CvBridge().imgmsg_to_cv2(msg, desired_encoding="passthrough")


    msg = Float32()
    # print "got image"

    # print cv_image.shape
    last_brightness = get_brightness(cv_image, True)

    msg.data = last_brightness
    pub.publish(msg)


if __name__ == "__main__":
    global pub

    full_image = True
    rospy.init_node("brightness_test")

    img_time = rospy.Time.now() + rospy.Duration(1e10)


    camera_name = "/pylon"
    service_name = camera_name+"/set_brightness_srv"

    rospy.Subscriber(camera_name+"/image_raw", Image, img_cb, queue_size = 1)
    pub = rospy.Publisher("/pylon/brightness", Float32, queue_size=10)


    rospy.spin()