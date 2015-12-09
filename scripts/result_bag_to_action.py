#! /usr/bin/env python


__author__ = 'klank'


import rospy
import camera_control_msgs.msg
import actionlib
import sensor_msgs.msg


class ImageReplicator(object):
    def __init__(self, action_name):
        self._action_name = action_name
        rospy.loginfo("open action server: " + str(action_name))

        self._as = actionlib.SimpleActionServer(self._action_name, camera_control_msgs.msg.GrabImagesAction,
                                                self.execute_cb, False)

        rospy.loginfo("subscribe to : /bag" + str(action_name) + "/result")

        self._sub = rospy.Subscriber("/bag"+str(action_name)+"/result",
                                     camera_control_msgs.msg.GrabImagesActionResult,
                                     self.image_callback, queue_size=5)

        rospy.loginfo("subscribe to : /bag/sol_camera/camera_info")

        self._sub = rospy.Subscriber("/bag/sol_camera/camera_info",
                                     sensor_msgs.msg.CameraInfo,
                                     self.cam_info_callback, queue_size=5)
        rospy.loginfo("publish: /bag/sol_camera/camera_info")

        self._pub = rospy.Publisher("/sol_camera/camera_info",
                                    sensor_msgs.msg.CameraInfo,
                                    queue_size=5, latch=True)

        self.image_list = []

        self._as.start()

    def cam_info_callback(self, msg):
        #rospy.loginfo("got caminfo")
        self._pub.publish(msg)

    def image_callback(self, msg):
        #rospy.loginfo("got image")
        self.image_list.append(msg)

    def execute_cb(self, goal):
        #rospy.loginfo("got action goal")
        while len(self.image_list) == 0:
            rospy.sleep(0.5)

        msg = self.image_list[0]

        if len(self.image_list) > 1:
            self.image_list = self.image_list[1:]
        else:
            self.image_list = []

        self._as.set_succeeded(msg.result)


def main():
    rospy.init_node("result_bag_to_action")
    ir = ImageReplicator("/sol_camera/grab_images_raw")
    rospy.spin()

if __name__ == '__main__':
    main()