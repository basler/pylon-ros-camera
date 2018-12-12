#! /usr/bin/python

import rospy
from std_srvs.srv import SetBoolRequest, SetBool
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2

from camera_control_msgs.srv import SetExposure, SetExposureRequest



rospy.init_node("flash_test")

# todo? check if auto_flash is activated for camera

camera_name = rospy.get_param("camera_name", "/pylon_camera_node")
exposure = rospy.get_param("exposure", 10000)
img_topic = camera_name + '/image_raw'
bridge = CvBridge()


# ensure current value (e.g. defaults) are visible on parameter server
rospy.set_param("exposure", exposure)
rospy.set_param("camera_name", camera_name)

if exposure > 0:
    rospy.loginfo("Setting exposure to %i ns" % exposure)

    exp_service_name = camera_name+'/set_exposure'
    exp_client = rospy.ServiceProxy(exp_service_name)
    if not exp_client.wait_for_service(3):
        rospy.logerr("No service at %s, terminating", exp_service_name)
        exit(1)
    print exp_client.call(target_exposure=exposure)

# establish service clients for outputs
clients = list()
for i in [0, 1]:
    srv_name = camera_name + "/activate_autoflash_output_" + str(i)
    client = rospy.ServiceProxy(srv_name, SetBool)
    if not client.wait_for_service(3):
        rospy.logerr("No service at %s, terminating", srv_name)
        exit(2)
    clients.append(client)

assert len(clients) == 2


# Create different light situations and capture image (turn both off at end to stop the party)
lights = [(1, 1), (0, 1), (1, 0), (0, 0)]

img_prefix = "/tmp/flash_test_"

rospy.loginfo("Writing images to %s_*.png", img_prefix)

for light in lights:
    rospy.loginfo("Setting lights to " + str(light))
    print clients[0].call(lights[0])
    print clients[1].call(lights[1])
    rospy.sleep(0.2) # needed??

    try:
      img = rospy.wait_for_message(img_topic, rospy.Duration(3))
    except rospy.exceptions.ROSException as e:
        rospy.logerr("Did not receive image at %s", img_topic)
        continue

    cv_img = bridge.imgmsg_to_cv2(img)
    print cv_img.shape()
    filename = img_prefix + "%i_%i.png" % (lights[0], lights[1])
    cv2.imwrite(filename, cv_img)

rospy.loginfo("Test ended")















