#!/usr/bin/env python

import rospy
from pp_msgs.srv import PathPlanningPlugin, PathPlanningPluginResponse
from geometry_msgs.msg import Twist
from gridviz import GridViz
from algorithms.astar import astar
from swarm_choosestation.msg import Publishpoint
from std_msgs.msg import String
import numpy as np

previous_plan_variables = None
# Start status
started = False
# Position clicked
x = []
y = []
optimalPath =  []

def make_plan(req):
  ''' 
  Callback function used by the service server to process
  requests from clients. It returns a msg of type PathPlanningPluginResponse
  ''' 
  global previous_plan_variables

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
  origin = [-10, -10, 0]
  if x is not None:
    d = np.zeros((len(x),len(y)))
    rospy.loginfo('A* started!!!!!')
    for i in range(0,len(x)):
      for j in range(0,len(y)):
        if i != j:
          start_position = (200+int(y[i]/0.05))*width + (200+int(x[i]/0.05))
          target_postion = (200+int(y[j]/0.05))*width + (200+int(x[j]/0.05))
          viz = GridViz(costmap, resolution, origin, start_position, target_postion, width)

          # time statistics
          start_time = rospy.Time.now()

          # calculate the shortes path
          path, previous_plan_variables = astar(start_position, target_postion, width, height, costmap, resolution, origin, viz, previous_plan_variables)

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
            
        optimalPath[i][j] = path
  else:
    path = []

  resp = PathPlanningPluginResponse()
  resp.plan = path
  return resp 

def clean_shutdown():
  cmd_vel.publish(Twist())
  rospy.sleep(1)

def getPublishpoint(data):
  global x,y
  x = data.posx
  y = data.posy
  

if __name__ == '__main__':
  rospy.init_node('path_planning_service_server', log_level=rospy.INFO, anonymous=False)
  rospy.Subscriber("publish_point", Publishpoint, getPublishpoint)
  make_plan_service = rospy.Service("/move_base/SrvClientPlugin/make_plan", PathPlanningPlugin, make_plan)
  cmd_vel = rospy.Publisher('sw1/cmd_vel', Twist, queue_size=5)
  rospy.on_shutdown(clean_shutdown)
  
  while not rospy.core.is_shutdown():
    rospy.rostime.wallsleep(0.5)
  rospy.Timer(rospy.Duration(2), rospy.signal_shutdown('Shutting down'), oneshot=True)
  rospy.spin()