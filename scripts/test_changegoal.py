#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import csv

x_odom = 0.0
y_odom = 0.0
x_goal = 0.0
y_goal = 0.0

rate = 0.5

x_stations = []
y_stations = []
package_path = '/home/swarm/catkin_ws/src/swarm_choosestation/csv/pos.csv'
fieldnames = ['x', 'y', 'z']
with open(package_path) as f:
  csv_reader = csv.DictReader(f, fieldnames)
  next(csv_reader)
  for line in csv_reader:
      x_stations.extend({line['x']})
      y_stations.extend({line['y']})
  # convert string list to float list
  x_stations = [float(x) for x in x_stations]
  y_stations = [float(y) for y in y_stations]
  rospy.sleep(1)

def odometryCb(msg):
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        x_odom = msg.pose.pose.position.x
        y_odom = msg.pose.pose.position.y
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.pose.position.x = x_stations
        goal_pose.pose.position.y = 0.0
        goal_pub.publish(goal_pose)
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node('oodometry', anonymous=True) #make node 
    rospy.Subscriber('/sw1/odom',Odometry,odometryCb)
    rospy.spin()