#!/usr/bin/env python

import rospy
from pp_msgs.srv import PathPlanningPlugin, PathPlanningPluginResponse
from geometry_msgs.msg import Twist, PoseStamped
from gridviz import GridViz
from algorithms.astar import astar

previous_plan_variables = None

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

  # x = [0,1]
  # y = [0,1]
  # i = 0
  # j= 1

  # start_index = (200+int(y[i]/0.05))*width + (200+int(x[i]/0.05))
  # goal_index = (200+int(y[j]/0.05))*width + (200+int(x[j]/0.05))

  viz = GridViz(costmap, resolution, origin, start_index, goal_index, width)

  # time statistics
  start_time = rospy.Time.now()
  # rospy.loginfo('start index: ' + str(start_index))
  # rospy.loginfo('goal index: ' + str(goal_index))
  # rospy.loginfo('width: ' + str(width))
  # rospy.loginfo('height: ' + str(height))
  # rospy.loginfo('resolutions: ' + str(resolution))
  # rospy.loginfo('start x: ' + str(start_index % width))
  # rospy.loginfo('start y: ' + str(int(start_index / width)))

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
    rospy.loginfo('**************Path cost:' + str(path_cost))

  resp = PathPlanningPluginResponse()
  resp.plan = path
  rospy.loginfo('Optimal path: ' + str(path))
  return resp

def clean_shutdown():
  cmd_vel.publish(Twist())
  rospy.sleep(1)

if __name__ == '__main__':
  rospy.init_node('path_planning_service_server', log_level=rospy.INFO, anonymous=False)
  make_plan_service = rospy.Service("sw1/move_base/SrvClientPlugin/make_plan", PathPlanningPlugin, make_plan)
  cmd_vel = rospy.Publisher('sw1/cmd_vel', Twist, queue_size=5)
  rospy.on_shutdown(clean_shutdown)

  while not rospy.core.is_shutdown():
    rospy.rostime.wallsleep(0.5)
  rospy.Timer(rospy.Duration(2), rospy.signal_shutdown('Shutting down'), oneshot=True)
