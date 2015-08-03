#!/usr/bin/env python  
import roslib
import rospy
import sys
from sensor_msgs.msg import Image
import datetime

global_start = 0

last_crane_stamp = 0
crane_timeout_ctr = 0
num_crane_imgs = 0

last_insertion_stamp = 0
insertion_timeout_ctr = 0
num_insertion_imgs = 0

def crane_img_cb(msg):
  global last_crane_stamp, crane_timeout_ctr, num_crane_imgs
  
  num_crane_imgs = num_crane_imgs + 1
  delta = rospy.get_time() - last_crane_stamp
  #print "got crane img! delta = " + str(delta)

  if delta > 0.5:
     if delta < 3.0:
       crane_timeout_ctr = crane_timeout_ctr + 1
       
  last_crane_stamp = rospy.get_time()

def insertion_img_cb(msg):
  global last_insertion_stamp, insertion_timeout_ctr, num_insertion_imgs
  
  num_insertion_imgs = num_insertion_imgs + 1
  delta = rospy.get_time() - last_insertion_stamp
  # print "got insertion img! delta = " + str(delta)

  if delta > 0.5:
    if delta < 3.0:
      insertion_timeout_ctr = insertion_timeout_ctr + 1
      
  last_insertion_stamp = rospy.get_time()
  
if __name__ == "__main__":
  #global last_crane_stamp, last_insertion_stamp, global_start 
  
  try:
    rospy.init_node("frame_rate_tester")

    last_crane_stamp = rospy.get_time()
    last_insertion_stamp = rospy.get_time()
    global_start = rospy.get_time()
    
      
    rospy.Subscriber("/crane/image_rect", Image, crane_img_cb, queue_size = 1)
    rospy.Subscriber("/pylon/image_rect", Image, insertion_img_cb, queue_size = 1)
      
    rospy.spin()
    
    f = open('/tmp/maru_frame_rate_tester_result.txt', 'a')
    f.write('###########################################################################\n')
    f.write('Result from '+ datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S") + '\n')
    f.write("Frame Rate Tester has finished after " + str(rospy.get_time() - global_start) + " seconds \n")
    f.write("CRANE: Recieved " + str(num_crane_imgs) + " images. Crane Cam exceeded timeout " + str(crane_timeout_ctr) + " times.\n")
    f.write("INSERTION: Recieved " + str(num_insertion_imgs) + " images. Insertion Cam exceeded timeout " + str(insertion_timeout_ctr) + " times.\n")
    f.write('###########################################################################\n')

#     print "\n"
#     print "Frame Rate Tester has finished after " + str(rospy.get_time() - global_start) + " seconds"
#     print "CRANE: Recieved " + str(num_crane_imgs) + " images. Crane Cam exceeded timeout " + str(crane_timeout_ctr) + " times."
#     print "INSERTION: Recieved " + str(num_insertion_imgs) + " images. Insertion Cam exceeded timeout " + str(insertion_timeout_ctr) + " times."
    
  except rospy.ROSInterruptException:
    print "Program interrupted before completion"