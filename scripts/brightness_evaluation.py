#! /usr/bin/python2
from django.template.defaultfilters import last

from pylon_camera_msgs.srv import SetBrightnessSrv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import rospy
import random

img_time = None #rospy.Time.now() + rospy.Duration(1e10)
servProxy = None

# last_brightness = 0


def get_brightness(img):
    global full_image

    #print "accepted"
    if full_image:
        # print 'full'
        return cv2.mean(img)[0]

    else:
        w,h = img.shape
        # r = img[0.25*w:0.75*w,0.25*h:0.75*h]
        return cv2.mean(img[0.25*w:0.75*w,0.25*h:0.75*h])[0]
        # cv2.imwrite('img.jpg',r)



def img_cb(msg):
    global last_brightness, img_time
    if rospy.Time.now() < img_time:
        #print "rejected"
        return
        #pass


    # print type(msg)
    cv_image = CvBridge().imgmsg_to_cv2(msg, desired_encoding="passthrough")

    # print cv_image.shape
    last_brightness = get_brightness(cv_image)
    #print "last", last_brightness



if __name__ == "__main__":
    global last_brightness, full_image

    full_image = True
    rospy.init_node("brightness_test")

    img_time = rospy.Time.now() + rospy.Duration(1e10)


    camera_name = "/pylon"
    service_name = camera_name+"/set_brightness_srv"

    rospy.Subscriber(camera_name+"/image_raw", Image, img_cb, queue_size = 1)

    rospy.wait_for_service(service_name)
    servProxy = rospy.ServiceProxy(service_name, SetBrightnessSrv)

    br_list = range(10, 61, 10)

    random.shuffle(br_list)

    do_next = True

    pos = 0
    expected = 0
    while  not rospy.is_shutdown():
        if do_next:
            expected = br_list[pos]
            print "requesting", expected
            if servProxy(expected):
                pos += 1
                full_image = not (expected < 50 or expected > 205)
                img_time = rospy.Time.now()#+rospy.Duration(5)
                last_brightness = 0
                do_next = False
            else:
                print "brightness failed"


        if last_brightness > 0:
            print "expected", expected, "got", last_brightness
            do_next = True
            if pos == len(br_list)-1:
                print "finished"
                exit(0)


        rospy.sleep(0.01)

    #rospy.spin()