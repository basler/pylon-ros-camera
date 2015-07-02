#!/usr/bin/env python  
import roslib
roslib.load_manifest('maru_msgs')
import rospy
import actionlib
import sys
import maru_msgs.msg
import pylon_camera_msgs.srv
from math import pi

class CompAction():
    def __init__(self, camera_name):
       self.camera_name = camera_name
       self._action_name = camera_name+'/calib_exposure_action'
       self._as = actionlib.SimpleActionServer(self._action_name, maru_msgs.msg.calib_exposureAction, execute_cb=self.execute_cb, auto_start = False)
       self.exp_srv = rospy.ServiceProxy(camera_name + '/set_exposure_srv', pylon_camera_msgs.srv.SetExposureSrv)
       self._result =  maru_msgs.msg.calib_exposureResult()
       self._feedback =  maru_msgs.msg.calib_exposureFeedback()
       self._as.start()
       
    def execute_cb(self, msg):
       rospy.loginfo("got exposure request for: " + str(msg.goal_exposure))
       self._as.publish_feedback(self._feedback)       
       self.exp_srv(msg.goal_exposure)
       self._as.set_succeeded(self._result)
       
    
    




if __name__ == '__main__':
  
  try:
      rospy.init_node('exp_service_for_compatibility')
      rospy.loginfo("starting action server for crane")
      as1 = CompAction("/crane")
      rospy.loginfo("starting action server for pylon")
      as2 = CompAction("/pylon")
      rospy.spin()
      # print result
      # print "Result:", ', '.join([str(n) for n in result.sequence])
  except rospy.ROSInterruptException:
      print "program interrupted before completion"
