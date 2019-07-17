#!/usr/bin/env python
import rospy
import csv
import rospkg
from nav_msgs.msg import Odometry
from icp_tracking.msg import * 
global path

def callback(objects_data):
  with open(path, 'ab') as csvfile:
    for i in range(len(objects_data.points)):
      time = objects_data.headers[i].stamp.secs
      ntime = objects_data.headers[i].stamp.nsecs
      writer = csv.writer(csvfile)
      n = "{:0>9d}".format(ntime)
      writer.writerow([str(time)+"."+n, objects_data.ids[i], objects_data.points[i].x, objects_data.points[i].y, objects_data.points[i].z])

def listener(): 
  rospy.init_node('get_gps', anonymous=True)
  rospy.Subscriber("/tracking_result", Objects ,callback)

if __name__ == '__main__':
  # print("123")
  listener()
  rospack = rospkg.RosPack()
  path = rospack.get_path('ground_truth') + "/../ground_truth/result.csv"
  with open(path, 'w') as csvfile:
    writer = csv.writer(csvfile)
  rospy.spin()
