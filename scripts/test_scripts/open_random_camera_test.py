#!/usr/bin/env python

__author__ = 'Nikolas Engelhard'

import rospy
import unittest

from sensor_msgs.msg import Image, CameraInfo


class CheckRandomCamera(unittest.TestCase):
    """
    This tests expects a camera node with name "/pylon_camera_node" that sends one the "image_raw"
    and the "camera_info" at least once within two seconds.
    """
    def setUp(self):
        rospy.init_node("open_camera_check")
        rospy.sleep(2)  # waiting for camera to be opened
        self.sub_image_ = rospy.Subscriber("/pylon_camera_node/image_raw", Image, self.image_callback, queue_size=1)
        self.sub_cam_info_ = rospy.Subscriber("/pylon_camera_node/camera_info", CameraInfo, self.cam_info_callback, queue_size=1)

        self.image_received_ = False
        self.calibration_received_ = False

    def image_callback(self, msg):
        assert isinstance(msg, Image)
        self.image_received_ = True

    def cam_info_callback(self, msg):
        assert isinstance(msg, CameraInfo)
        self.calibration_received_ = True

    # def test2(self):
    #     self.assertTrue(True)

    def test_wait_for_data(self):
        all_topics = rospy.get_published_topics()
        self.assertTrue(len(all_topics) > 2, "No topics found")
        names = [t[0] for t in all_topics]
        self.assertTrue("/pylon_camera_node/image_raw" in names, names)
        self.assertTrue("/pylon_camera_node/camera_info" in names)

        timeout_duration_sec = 2.0
        timeout = rospy.Time.now() + rospy.Duration(timeout_duration_sec)

        while not rospy.is_shutdown() and rospy.Time.now() < timeout:
            if self.image_received_ and self.calibration_received_:
                return
            rospy.sleep(0.1)

        if not self.image_received_:
            self.assertTrue(False, ("No image received within %.0f seconds" % timeout_duration_sec))
        if not self.calibration_received_:
            self.assertTrue(False, ("No calibration data received within %.0f seconds" % timeout_duration_sec))

        print self.image_received_
        print self.calibration_received_


if __name__ == '__main__':
    import rostest
    rostest.rosrun("pylon_camera", 'pylon', CheckRandomCamera)
