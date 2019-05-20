#!/usr/bin/python
import sys, rospy, time
import numpy as np
from master_msgs_iele3338.srv import mapaInicio
from master_msgs_iele3338.msg import Obstacle
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import *

if __name__ == '__main__':
	rospy.init_node('mapa')
	rospy.wait_for_service('mapa_inicio')
	met = rospy.ServiceProxy('mapa_inicio', mapaInicio)
	req = met()
	start = np.array([req.start.position.x, req.start.position.y, req.start.orientation.w])
	goal = np.array([req.goal.position.x, req.goal.position.y, req.goal.orientation.w])
	obstacles = []
	n_obstacles = req.n_obstacles
	for i in range(req.n_obstacles):
		obstacles.append(np.array([req.obstacles[i].position.position.x , req.obstacles[i].position.position.y, req.obstacles[i].radius]))
 	print(obstacles)