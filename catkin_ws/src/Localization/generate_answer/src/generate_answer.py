#!/usr/bin/env python

import rospy
import csv
import rospkg
from nav_msgs.msg import Odometry

global path

def callback(odom_data):
  with open(path, 'ab') as csvfile:
    time = odom_data.header.stamp.secs()
    ntime = odom_data.header.stamp.nsecs()
    writer = csv.writer(csvfile)
    writer.writerow([str(time)+"."+str(ntime), odom_data.pose.pose.position.x, odom_data.pose.pose.position.y, odom_data.pose.pose.position.z])

def listener(): 
  rospy.init_node('generate_answer', anonymous=True)
  rospy.Subscriber("/pc_loc_path", Odometry ,callback)
  #rospy.Subscriber("/zed/odom", Odometry ,callback)

if __name__ == '__main__':
  listener()
  rospack = rospkg.RosPack()
  path = rospack.get_path('generate_answer') + "/../localization_ground_truth/answer.csv"
  with open(path, 'w') as csvfile:
    writer = csv.writer(csvfile)
  rospy.spin()


