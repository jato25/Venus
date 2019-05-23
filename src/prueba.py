#!/usr/bin/python
import sys, rospy, time
import numpy as np
from master_msgs_iele3338.srv import StartService, AckService, EndService, StartServiceResponse, mapaInicio, mapaInicioResponse
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from master_msgs_iele3338.msg import Obstacle
from std_msgs.msg import *

ip = "157.253.110.177"
grupo = 6

def mapaInfo(req):
	global start, goal, obstacles, n_obstacles
	start = req.start
	goal = req.goal
	obstacles = req.obstacles
	n_obstacles = req.n_obstacles
	rospy.loginfo('Mapa recicibido')
	s.shutdown()
	return StartServiceResponse()

def inicioMapa(info):
	global start, goal, obstacles, n_obstacles
	s.shutdown()
	return mapaInicioResponse(start, goal, n_obstacles, obstacles)

if __name__ == '__main__':
	rospy.init_node('roberta' , anonymous = True)
	estado = 0
	rospy.wait_for_service('ack_service')
	ack = rospy.ServiceProxy('ack_service', AckService)
	while estado == 0:
		estado = ack(grupo , ip).state
	rospy.loginfo('Esperando servicio')
	s = rospy.Service('start_service', StartService,  mapaInfo)
	s.spin()
	s = rospy.Service('mapa_inicio', mapaInicio, inicioMapa)
	s.spin()
	tasa = rospy.Rate(50)
	while not rospy.is_shutdown():
		tasa.sleep()
