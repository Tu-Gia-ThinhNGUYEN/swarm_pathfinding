#!/usr/bin/env python

import rospy
from pp_msgs.srv import PathPlanningPlugin, PathPlanningPluginResponse
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from gridviz import GridViz
from algorithms.astar import astar
import numpy as np
from algorithms.ant_colony import AntColony
import csv
import json

previous_plan_variables = None
count = 0
optimalPath = []
x_odom = 0.0
y_odom = 0.0
initial_path = []
runningPath = 0
initial_pathcost = 0.0

pos_csv = '/home/swarm/catkin_ws/src/swarm_choosestation/csv/pos.csv'
path_package = '/home/swarm/catkin_ws/src/swarm_pathfinding'
path_distanceMatrix = path_package+'/json/distanceMatrix.json'
path_pathTemp = path_package + '/json/pathTemp.json'
path_pathArray = path_package + '/json/pathArray.json'
path_pathComponent = path_package + '/json/pathComponent.json'
path_opttimalPath = path_package + '/json/opttimalPath.json'

def odometryCb(msg):
  global x_odom, y_odom
  x_odom = msg.pose.pose.position.x
  y_odom = msg.pose.pose.position.y

def make_plan(req):
  ''' 
  Callback function used by the service server to process
  requests from clients. It returns a msg of type PathPlanningPluginResponse
  ''' 
  global previous_plan_variables, optimalPath, initial_path, runningPath, x, y, numberOfStations, path_array, initial_pathcost
  # costmap as 1-D array representation
  costmap = req.costmap_ros
  # number of columns in the occupancy grid
  width = req.width
  # number of rows in the occupancy grid
  height = req.height
  start_index = req.start
  goal_index = req.goal
  # side of each grid map square in meters
  resolution = 0.05
  # origin of grid map
  origin = [-9.6, -9.6, 0]
  x = []
  y = []
  fieldnames = ['x', 'y', 'z']
  with open(pos_csv) as f:
    csv_reader = csv.DictReader(f, fieldnames)
    next(csv_reader)
    for line in csv_reader:
        x.extend({line['x']})
        y.extend({line['y']})
    # convert string list to float list
    x = [float(x) for x in x]
    y = [float(y) for y in y]
    rospy.sleep(1)
  numberOfStations = len(x)
  json_distanceMatrix = {"d"+str(i)+str(j): [] for i in range(1, numberOfStations) for j in range(1, numberOfStations)}
  json_pathTemp = {"p"+str(i)+str(j): [] for i in range(1, numberOfStations) for j in range(1, numberOfStations)}
  json_pathComponent = {"p"+str(i): [] for i in range(1, numberOfStations)}
  json_opttimalPath = {"p": []}
  json_pathArray = {"p": []}
  rospy.loginfo('Data read from file ' + str(pos_csv))
  rospy.loginfo('Data read complete!')
  rospy.loginfo('x: ' + str(x))
  rospy.loginfo('y: ' + str(y))
  pathTemp = [[[] for _ in range(len(x))] for _ in range(len(x))]
  d = np.zeros((len(x),len(x)))
  rospy.loginfo('**********************Start*****************')
  # First run to find path from robot to first node
  start_index = int((round(abs(origin[0]/resolution))+int(y_odom/0.05))*width + (round(abs(origin[1]/resolution))+int(x_odom/0.05)))
  goal_index = int((round(abs(origin[0]/resolution))+int(y[0]/0.05))*width + (round(abs(origin[1]/resolution))+int(x[0]/0.05)))
  # rospy.loginfo("start :" + str(start_index) + " " + str(type(start_index)))
  # rospy.loginfo("start :" + str(goal_index) + " " + str(type(goal_index)))
  viz = GridViz(costmap, resolution, origin, start_index, goal_index, width)
  # time statistics
  start_time = rospy.Time.now()
  # calculate the shortes path
  path, previous_plan_variables, path_cost = astar(start_index, goal_index, width, height, costmap, resolution, origin, viz, previous_plan_variables)
  if not path:
    rospy.logwarn("No initial path returned by the path algorithm")
    path = []
  else:
    execution_time = rospy.Time.now() - start_time
    print("\n")
    rospy.loginfo('++++++++ Path Planning execution metrics ++++++++')
    rospy.loginfo('Total execution time: %s seconds', str(execution_time.to_sec()))
    rospy.loginfo('++++++++++++++++++++++++++++++++++++++++++++')
    print("\n")
    rospy.loginfo('Path sent to navigation stack')
    initial_pathcost = path_cost
    initial_path = path
  # try:
  #   rospy.loginfo("Check data in json file...")
  #   # Opening JSON file
  #   with open(path_pathComponent) as openfile:
  #     # Reading from json file
  #     json_object = json.load(openfile)
  #   rospy.sleep(1)
  #   for i in range(0, len(json_object)):
  #     stage_optimalPath.append(json_object["p"+str(i+1)])
  #   # Opening JSON file
  #   with open(path_pathArray) as openfile:
  #     # Reading from json file
  #     json_object = json.load(openfile)
  #   rospy.sleep(1)
  #   path_array = json_object["p"]
  #   rospy.loginfo("Read data in json file successfully!!!!!!!!!!!!")
  # except:
  rospy.loginfo('**********************A* Start *****************')
  for i in range(0, len(x)):
    for j in range(0, len(y)):
      if i != j:
        start_index = int((round(abs(origin[0]/resolution))+int(y[i]/0.05))*width + (round(abs(origin[1]/resolution))+int(x[i]/0.05)))
        goal_index = int((round(abs(origin[0]/resolution))+int(y[j]/0.05))*width + (round(abs(origin[1]/resolution))+int(x[j]/0.05)))
        viz = GridViz(costmap, resolution, origin, start_index, goal_index, width)
        # time statistics
        start_time = rospy.Time.now()
        # calculate the shortes path
        path, previous_plan_variables, path_cost = astar(start_index, goal_index, width, height, costmap, resolution, origin, viz, previous_plan_variables)
        if not path:
          rospy.logwarn("No path returned by the path algorithm")
          path = []
        else:
          execution_time = rospy.Time.now() - start_time
          print("\n")
          rospy.loginfo('++++++++ Path Planning execution metrics ++++++++')
          rospy.loginfo('Total execution time: %s seconds', str(execution_time.to_sec()))
          rospy.loginfo('++++++++++++++++++++++++++++++++++++++++++++')
          print("\n")
          rospy.loginfo('Path sent to navigation stack')
          d[i,j] = path_cost
          json_distanceMatrix["d"+str(i+1)+str(j+1)] = path_cost
          pathTemp[i][j] = path
          json_pathTemp["p"+str(i+1)+str(j+1)] = path
      else:
        d[i,j] = np.inf
  rospy.loginfo('Distance matrix: ' + str(d))
  rospy.loginfo('******** A* Done ********')
  rospy.loginfo('########### Ant Colony Optimization Start ###########')
  ant_colony = AntColony(d, 100, 20, 100, 0.95, alpha=1, beta=1)
  path_array = ant_colony.run()
  json_pathArray["p"] = path_array
  print (path_array)
  shortestPath = []
  for i in range(0,len(path_array[0])):
      shortestPath.extend(pathTemp[path_array[0][i][0]][path_array[0][i][1]])
      json_pathComponent["p"+str(i+1)] = pathTemp[path_array[0][i][0]][path_array[0][i][1]]
  optimalPath = shortestPath
  json_opttimalPath["p"] = shortestPath
  # Serializing json
  object_distanceMatrix = json.dumps(json_distanceMatrix, indent=numberOfStations**2)
  object_pathTemp = json.dumps(json_pathTemp, indent=numberOfStations**2)
  object_pathArray = json.dumps(json_pathArray, indent=1)
  object_pathComponent = json.dumps(json_pathComponent, indent=numberOfStations)
  object_opttimalPath = json.dumps(json_opttimalPath, indent=1)
  # Writing to sample.json
  with open(path_distanceMatrix, "w") as outfile:
    outfile.write(object_distanceMatrix)
  with open(path_pathTemp, "w") as outfile:
    outfile.write(object_pathTemp)
  with open(path_pathArray, "w") as outfile:
    outfile.write(object_pathArray)
  with open(path_pathComponent, "w") as outfile:
    outfile.write(object_pathComponent)
  with open(path_opttimalPath, "w") as outfile:
    outfile.write(object_opttimalPath)
  rospy.loginfo('Optimal path: ' + str(shortestPath))
  rospy.loginfo('Optimal path cost: ' + str(path_array[1]))
  rospy.loginfo('Initial path cost: ' + str(initial_pathcost))
  rospy.loginfo('Total of path cost: ' + str(path_array[1]+initial_pathcost))
  rospy.loginfo('########### Ant Colony Optimization Done ###########')

  resp = PathPlanningPluginResponse()
  resp.plan = initial_path
  rospy.signal_shutdown(clean_shutdown)
  return resp
  

def clean_shutdown():
  cmd_vel.publish(Twist())
  rospy.sleep(1)
  rospy.loginfo("Database has been saved at: " + path_package)

if __name__ == '__main__':
  rospy.init_node('path_planning_service_server', log_level=rospy.INFO, anonymous=False)
  rospy.Subscriber('/sw1/odom',Odometry,odometryCb)
  
  make_plan_service = rospy.Service("/move_base/SrvClientPlugin/make_plan", PathPlanningPlugin, make_plan)
  cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
  
  rospy.on_shutdown(clean_shutdown)
  # rospy.signal_shutdown(clean_shutdown)

  while not rospy.core.is_shutdown():
    rospy.rostime.wallsleep(0.5)
  rospy.Timer(rospy.Duration(2), rospy.signal_shutdown('Shutting down'), oneshot=True)
