#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import csv
import json
import math
import numpy as np

x_stations = []
y_stations = []
package_path = '/home/swarmpc/catkin_ws/src/swarm_choosestation/csv/pos.csv'
path_package = '/home/swarmpc/catkin_ws/src/swarm_pathfinding'
path_pathArray = path_package + '/json/pathArray.json'
fieldnames = ['x', 'y', 'z']
stations = []
index = 0

with open(path_pathArray, 'r') as openfile:
    # Reading from json file
    json_object = json.load(openfile)
for i in range(0, len(json_object)):
    stations.append(json_object["p"])

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

def movebase_client(index):
    if (index == -1):
        index = 0
        x_nextstation = x_stations[index]
        y_nextstation = x_stations[index]
        x_currentsation = x_stations[index]
        y_currentsation = y_stations[index] 
    elif (index < len(x_stations)-1):
        x_nextstation = x_stations[index+1]
        y_nextstation = y_stations[index+1]
        x_currentsation = x_stations[index]
        y_currentsation = y_stations[index]
    else:
        x_nextstation = x_stations[0]
        y_nextstation = y_stations[0]
        x_currentsation = x_stations[index]
        y_currentsation = y_stations[index]    

    roll = 0.0
    pitch = 0.0
    yaw = math.atan2(y_nextstation-y_currentsation,x_nextstation-x_currentsation)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    client = actionlib.SimpleActionClient('sw1/move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x_currentsation
    goal.target_pose.pose.position.y = y_currentsation
    goal.target_pose.pose.orientation.w = qw
    rospy.loginfo("Station is being excuted:")
    rospy.loginfo("x: " + str(x_currentsation)+"       y: "+str(y_currentsation)+"     qw: "+str(qw))
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
        return False
    else:
        return client.get_result()

print("TSP Start...")
print("Stations: " + str(stations))

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        # while index < len(stations[0][0]):
        #     result = movebase_client(stations[0][0][index][0])
        #     if result:
        #         rospy.loginfo("Station " + str(stations[0][0][index][0]+1) + " execution done!")
        #         index = index + 1
        #     else:
        #         rospy.loginfo("TSP fail")
        #         break
        result = movebase_client(0)
        if result:
            rospy.loginfo("TSP done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")