#!/usr/bin/python
import sys, rospy, time
import numpy as np
from master_msgs_iele3338.srv import StartService, AckService, EndService, StartServiceResponse
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from master_msgs_iele3338.msg import Obstacle
from std_msgs.msg import *

ip = "157.253.110.177"
grupo = 6

def mapaInfo(req):
	global start, goal, obstacles
	start = np.array([req.start.position.x, req.start.position.y, req.start.orientation.w])
	goal = np.array([req.goal.position.x, req.goal.position.y, req.goal.orientation.w])
	obstacles = []
	for i in range(req.n_obstacles):
		obstacles.append(np.array([req.obstacles[i].position.position.x , req.obstacles[i].position.position.y, req.obstacles[i].radius]))
	print('Mapa recicibido')
	s.shutdown()
	return StartServiceResponse()
	
if __name__ == '__main__':
	rospy.init_node('roberta' , anonymous = True)
	estado = 0
	rospy.wait_for_service('ack_service')
	ack = rospy.ServiceProxy('ack_service', AckService)
	while estado == 0:
		estado = ack(grupo , ip).state
	print('Esperando servicio')
	s = rospy.Service('start_service', StartService,  mapaInfo)
	s.spin()
	tasa = rospy.Rate(10)
	while not rospy.is_shutdown():
		tasa.sleep()
