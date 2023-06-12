#!/usr/bin/env python

import rospy
from pp_msgs.srv import PathPlanningPlugin, PathPlanningPluginResponse
from geometry_msgs.msg import Twist, PoseStamped
from gridviz import GridViz
from algorithms.astar import astar
import numpy as np
from algorithms.ant_colony import AntColony

previous_plan_variables = None
count = 0

def make_plan(req):
  ''' 
  Callback function used by the service server to process
  requests from clients. It returns a msg of type PathPlanningPluginResponse
  ''' 
  global count
  count = count + 1
  rospy.loginfo('CHAY LAN ' + str(count))
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

  # x = [1.9366612434387207, -3.053391933441162, -3.0622968673706055, 0.30975770950317383, 2.032531261444092]
  # y = [3.9708738327026367, 3.220120429992676, -2.63504695892334, -2.9622130393981934, -1.0816521644592285]
  x = [1.9366612434387207, -3.053391933441162, -3.0622968673706055]
  y = [3.9708738327026367, 3.220120429992676, -2.63504695892334]
  optimalPath = [[[] for _ in range(len(x))] for _ in range(len(x))]
  d = np.zeros((len(x),len(x)))
  rospy.loginfo('******** A* Start ********')
  for i in range(0, len(x)):
    for j in range(0, len(y)):
      if i != j:

        start_index = (200+int(y[i]/0.05))*width + (200+int(x[i]/0.05))
        goal_index = (200+int(y[j]/0.05))*width + (200+int(x[j]/0.05))

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
          d[i,j] = path_cost
          optimalPath[i][j] = path
      else:
        d[i,j] = np.inf
  rospy.loginfo('Distance matrix: ' + str(d))
  rospy.loginfo('******** A* Done ********')

  rospy.loginfo('########### Ant Colony Optimization Start ###########')
  ant_colony = AntColony(d, 100, 20, 100, 0.95, alpha=1, beta=1)
  path_array = ant_colony.run()
  print (path_array)
  shortestPath = []
  for i in range(0,len(path_array[0])):
      shortestPath.extend(optimalPath[path_array[0][i][0]][path_array[0][i][1]])

  shortestPath = [106989, 106604, 106219, 105834, 105449, 105064, 104679, 104294, 103909, 103524, 103139, 102754, 102369, 102368, 102367, 102366, 102365, 102364, 102363, 102362, 102361, 102360, 102359, 102358, 102357, 102356, 101971, 101970, 101969, 101968, 101967, 101966, 101965, 101964, 101963, 101962, 101961, 101960, 101959, 101958, 101957, 101956, 101955, 101954, 101953, 101952, 101951, 101950, 101949, 101948, 101947, 101946, 101945, 101944, 101943, 101942, 101941, 101940, 101939, 101938, 101937, 101936, 101935, 101934, 101933, 101932, 101931, 101930, 101929, 101928, 101927, 101926, 101925, 101924, 101539, 101538, 101537, 101536, 101535, 101534, 101533, 101532, 101531, 101530, 101529, 101528, 101527, 101526, 101525, 101524, 101523, 101522, 101521, 101520, 101519, 101518, 101517, 101516, 101515, 101515, 101131, 100747, 100363, 99979, 99595, 99211, 98827, 98443, 98059, 97675, 97291, 96907, 96523, 96139, 95755, 95371, 94987, 94603, 94219, 93835, 93451, 93067, 92683, 92299, 91915, 91531, 91147, 90763, 90379, 89995, 89611, 89227, 88843, 88459, 88075, 87691, 87307, 86923, 86539, 86155, 85771, 85387, 85003, 84619, 84235, 83851, 83467, 83083, 82699, 82315, 81931, 81547, 81163, 80779, 80395, 80011, 79627, 79243, 78859, 78475, 78091, 77707, 77323, 76939, 76555, 76171, 75787, 75403, 75019, 74635, 74251, 73867, 73483, 73099, 72715, 72331, 71947, 71563, 71179, 70795, 70411, 70027, 69643, 69259, 68875, 68491, 68107, 67723, 67339, 66955, 66571, 66187, 65803, 65419, 65035, 64651, 64267, 63883, 63499, 63115, 62731, 62347, 61963, 61579, 61195, 60811, 60427, 60043, 59659, 59275, 58891, 58507, 58123, 57739, 57355, 56971, 56971, 57356, 57741, 58126, 58511, 58896, 59281, 59665, 60048, 60432, 60815, 61199, 61583, 61967, 62351, 62735, 63119, 63503, 63887, 64271, 64655, 65040, 65424, 65809, 66193, 66578, 66963, 67347, 67731, 68115, 68499, 68883, 69267, 69651, 70034, 70417, 70800, 71184, 71567, 71951, 72334, 72718, 73102, 73486, 73870, 74254, 74638, 75022, 75406, 75790, 76174, 76558, 76943, 77327, 77712, 78096, 78481, 78866, 79250, 79634, 80018, 80402, 80786, 81169, 81553, 81937, 82320, 82703, 83086, 83470, 83853, 84237, 84620, 85004, 85388, 85772, 86156, 86540, 86924, 87308, 87692, 88076, 88460, 88844, 89228, 89612, 89996, 90380, 90764, 91148, 91532, 91916, 92299, 92683, 93067, 93451, 93835, 94220, 94604, 94989, 95373, 95758, 96143, 96528, 96913, 97298, 97683, 98068, 98453, 98838, 99223, 99608, 99993, 100378, 100763, 101148, 101533, 101918, 102303, 102688, 103073, 103458, 103843, 104228, 104613, 104998, 105383, 105768, 106153, 106538, 106923, 106924, 106925, 106926, 106927, 106928, 106929, 106930, 106931, 106932, 106933, 106934, 106935, 106936, 106937, 106938, 106939, 106940, 106941, 106942, 106943, 106944, 106945, 106946, 106947, 106948, 106949, 106950, 106951, 106952, 106953, 106954, 106955, 106956, 106957, 106958, 106959, 106960, 106961, 106962, 106963, 106964, 106965, 107350, 107351, 107352, 107353, 107354, 107355, 107356, 107357, 107358, 107359, 107360, 107361, 107362, 107363, 107364, 107365, 107366, 107367, 107368, 107369, 107370, 107371, 107372, 107373, 107374, 107374]
  resp = PathPlanningPluginResponse()
  resp.plan = shortestPath
  rospy.loginfo('Optimal path: ' + str(shortestPath))
  rospy.loginfo('########### Ant Colony Optimization Done ###########')
  return resp

def clean_shutdown():
  cmd_vel.publish(Twist())
  rospy.sleep(1)

if __name__ == '__main__':
  rospy.init_node('path_planning_service_server', log_level=rospy.INFO, anonymous=False)
  make_plan_service = rospy.Service("/move_base/SrvClientPlugin/make_plan", PathPlanningPlugin, make_plan)
  cmd_vel = rospy.Publisher('sw1/cmd_vel', Twist, queue_size=5)
  rospy.on_shutdown(clean_shutdown)

  while not rospy.core.is_shutdown():
    rospy.rostime.wallsleep(0.5)
  rospy.Timer(rospy.Duration(2), rospy.signal_shutdown('Shutting down'), oneshot=True)
