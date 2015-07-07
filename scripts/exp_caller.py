#!/usr/bin/env python  
import roslib
roslib.load_manifest('maru_msgs')
import rospy
import actionlib
import sys
import maru_msgs.msg
from math import pi


def exposure_client(exp,camera_name):
    name = camera_name+'/calib_exposure_action'
    rospy.loginfo("waiting for server " + name)
    client_exp = actionlib.SimpleActionClient(name, maru_msgs.msg.calib_exposureAction)
    if not client_exp.wait_for_server(rospy.Duration(5.0)):
        rospy.logerr("Could not find server")
        exit(0)

    goal = maru_msgs.msg.calib_exposureGoal()
    goal.goal_exposure = exp
    goal.accuracy = 5
    client_exp.send_goal(goal)
    res = client_exp.wait_for_result(rospy.Duration(20.0))
    return res



if __name__ == '__main__':

    try:
        rospy.init_node('exp_caller')


        if len(sys.argv) < 3:
            print "Usage: ./exp_caller brightness camera"
            exit(0)

        cam_name = sys.argv[2]

        try:
            brightness = int(sys.argv[1])
        except ValueError:
            rospy.logerr("target brightness of " + str(sys.argv[1]) + " is not an integer!")
            exit(0)

        result = exposure_client(brightness,cam_name)

    except rospy.ROSInterruptException:
        print "program interrupted before completion"
