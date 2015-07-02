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
    print "waiting for server", name
    client_exp = actionlib.SimpleActionClient(name, maru_msgs.msg.calib_exposureAction)
    client_exp.wait_for_server()
    print "has server"
    goal = maru_msgs.msg.calib_exposureGoal()
    goal.goal_exposure = exp
    goal.accuracy = 5
    client_exp.send_goal(goal)
    res = client_exp.wait_for_result(rospy.Duration(20.0))
    return res



if __name__ == '__main__':
  
  try:
      rospy.init_node('exp_caller')
      cam_name = "/crane"
      if len(sys.argv) > 2:
          cam_name = sys.argv[2]
  
      result = exposure_client(int(sys.argv[1]),cam_name)
      # print result
      # print "Result:", ', '.join([str(n) for n in result.sequence])
  except rospy.ROSInterruptException:
      print "program interrupted before completion"
