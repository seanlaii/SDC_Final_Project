#!/usr/bin/env python
import rospy
import csv
import rospkg
from nav_msgs.msg import Odometry

global path

def callback(odom_data):
  with open(path, 'ab') as csvfile:
    time = odom_data.header.stamp.secs
    ntime = odom_data.header.stamp.nsecs
    writer = csv.writer(csvfile)
    writer.writerow([str(time)+"."+str(ntime), odom_data.pose.pose.position.x, odom_data.pose.pose.position.y, odom_data.pose.pose.position.z])

def listener(): 
  rospy.init_node('get_gps', anonymous=True)
  rospy.Subscriber("/pose_icp", Odometry ,callback)

if __name__ == '__main__':
  # print("123")
  listener()
  rospack = rospkg.RosPack()
  path = rospack.get_path('generate_answer') + "/../localization_ground_truth/icp1.csv"
  with open(path, 'w') as csvfile:
    writer = csv.writer(csvfile)
  rospy.spin()
