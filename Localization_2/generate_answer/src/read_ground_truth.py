#!/usr/bin/env python
import rospy
import csv
import rospkg
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry

global path
mmarker = Marker()

def readcsv(): 
  
  for line in reader:
    point = Point()
    point.x = float(line[1])
    point.y = float(line[2])
    point.z = float(line[3])
    mmarker.points.append(point) 
    pub.publish(mmarker)
  while not rospy.is_shutdown():
    pub.publish(mmarker)
  csvfile.close()
  
  
def marker():
  mmarker.header.frame_id = "/map"
  mmarker.header.stamp = rospy.get_rostime()
  mmarker.ns = "lines"
  mmarker.id = 2
  mmarker.type = mmarker.LINE_STRIP
  mmarker.action = mmarker.ADD
  mmarker.pose.position.x = 0
  mmarker.pose.position.y = 0
  mmarker.pose.position.z = 0
  mmarker.lifetime = rospy.Duration()
  mmarker.pose.orientation.x = 0
  mmarker.pose.orientation.y = 0
  mmarker.pose.orientation.z = 0
  mmarker.pose.orientation.w = 1.0
  mmarker.scale.x = 1.0
  mmarker.scale.y = 1.0
  mmarker.scale.z = 1.0

  mmarker.color.r = 0.0
  mmarker.color.g = 1.0
  mmarker.color.b = 0.0
  mmarker.color.a = 1.0 
  mmarker.points = []

def callback(odom_data):
  start = 1

if __name__ == '__main__':
  rospy.init_node('ground_truth', anonymous=True)
  pub = rospy.Publisher('ground_truth', Marker, queue_size=1)
  rospy.Subscriber("/pose_icp", Odometry ,callback)
  marker()
  rospack = rospkg.RosPack()
  path = rospack.get_path('generate_answer') + "/../localization_ground_truth/localization_ground_truth_1.csv"
  print(path)
  csvfile=open(path,'r')
  reader = csv.reader(csvfile)
  readcsv()
  rospy.spin()
