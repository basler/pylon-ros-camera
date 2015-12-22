#!/usr/bin/env python

__author__ = 'Nikolas Engelhard'

import rospy
import unittest

from sensor_msgs.msg import Image, CameraInfo


class SingleAcquisitionTest(unittest.TestCase):
    """
    This tests expects a camera node with a parametrized name to send "image_raw"
    and "camera_info" at least once within fife seconds.
    """
    def setUp(self):
        rospy.init_node('single_acquisition_test_node')

        # get parameter from the test-launch-file
        self.camera_frame = rospy.get_param('~camera_frame','')
        self.assertTrue(self.camera_frame,
                        'No camera frame given in the test-launch file, can not execute the test')

        self.sub_image_ = rospy.Subscriber("{}/image_raw".format(self.camera_frame),
                                           Image,
                                           self.imageCallback,
                                           queue_size=1)

        self.sub_cam_info_ = rospy.Subscriber("{}/camera_info".format(self.camera_frame),
                                              CameraInfo,
                                              self.camInfoCallback,
                                              queue_size=1)

        self.image_received_ = False
        self.calibration_received_ = False

    def imageCallback(self, msg):
        self.assertTrue(isinstance(msg, Image))
        self.image_received_ = True

    def camInfoCallback(self, msg):
        self.assertTrue(isinstance(msg, CameraInfo))
        self.calibration_received_ = True

    def testWaitForAcquisitionStarted(self):

        timeout_duration_sec = 5.0
        timeout = rospy.Time.now() + rospy.Duration(timeout_duration_sec)

        while not rospy.is_shutdown() and rospy.Time.now() < timeout:
            if self.image_received_ and self.calibration_received_:
                return
            rospy.sleep(0.2)

        if not self.image_received_:
            self.assertTrue(False, ("No image received within %.0f seconds" % timeout_duration_sec))
        if not self.calibration_received_:
            self.assertTrue(False, ("No calibration data received within %.0f seconds" % timeout_duration_sec))

if __name__ == '__main__':
    import rostest
    rostest.rosrun('pylon_camera', 'single_acquisition_test', SingleAcquisitionTest)
