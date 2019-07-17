#!/usr/bin/env python
import rospy
import csv
import rospkg
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry

global path
mmarker = Marker()
points = []
global count
def readcsv(): 
  
  for line in reader:
    point = Odometry()
    point.header.stamp = float(line[0])
    point.pose.pose.position.x = float(line[2])
    point.pose.pose.position.y = float(line[3])
    point.pose.pose.position.z = float(line[4])
    points.append(point);
    #mmarker.points.append(point) 
    #pub.publish(mmarker)
  #while not rospy.is_shutdown():
    #pub.publish(mmarker)
  csvfile.close()
  
  
def marker():
  mmarker.header.frame_id = "/velodyne"
  mmarker.header.stamp = rospy.get_rostime()
  mmarker.ns = "p"
  mmarker.id = 2
  mmarker.type = mmarker.SPHERE_LIST
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
  mmarker.color.g = 0.0
  mmarker.color.b = 1.0
  mmarker.color.a = 1.0 
  mmarker.points = []

def callback(odom_data):
  global count
  i = 0
  mmarker.points = []
  while points[count % len(points)].header.stamp == points[(count + i) % len(points)].header.stamp :
    point = Point()
    point.x = points[(count + i) % len(points)].pose.pose.position.x
    point.y = points[(count + i) % len(points)].pose.pose.position.y
    point.z = points[(count + i) % len(points)].pose.pose.position.z
    mmarker.points.append(point)
    i += 1
  #print(points[count % len(points)].header.stamp)
  count = (count + i) % len(points)
  print(count)
  pub.publish(mmarker)
    
if __name__ == '__main__':
  count = 0 
  rospy.init_node('ground_truth', anonymous=True)
  pub = rospy.Publisher('ground_truth', Marker, queue_size=1)
  #rospy.Subscriber("/pose_icp", Odometry ,callback)
  rospy.Subscriber("visualization_marker", Marker ,callback)
  marker()
  rospack = rospkg.RosPack()
  path = rospack.get_path('ground_truth') + "/config/tracking_ground_truth_1.csv"
  print(path)
  csvfile=open(path,'r')
  reader = csv.reader(csvfile)
  readcsv()
  rospy.spin()
