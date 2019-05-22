#!/usr/bin/python
import rospy
import numpy as np
from pylab import math
from std_msgs.msg import *
from master_msgs_iele3338.msg import Covariance
from geometry_msgs.msg import Pose, Point, Quaternion
from master_msgs_iele3338.srv import posInicio

pub = rospy.Publisher('robot_uncertainty', Covariance, queue_size=10)
pub1 = rospy.Publisher('robot_position', Pose, queue_size=10)
pub2 = rospy.Publisher('venus_position', Float32MultiArray, queue_size=10)
b = 90.0
r = 32.5	

def callback(data):
	global derAct
	derAct = data.data
	posicion()

def callback2(data):
	global izqAct
	izqAct = data.data
	posicion()

def callback3(data):
	global dirDer
	dirDer = data.data

def callback4(data):
	global dirIzq
	dirIzq = data.data

def posicion():
	global izqAct, derAct, izqAnt, derAnt, dirDer, dirIzq, pos, cov
	dizq = izqAct - izqAnt
	dder = derAct - derAnt
	izqAnt = izqAct
	derAnt = derAct
	dizq = 2*np.pi*dizq*r/(2*442)
	dder = 2*np.pi*dder*r/(2*442)
	if (dirDer == 0):
		dder = -dder
	if (dirIzq == 0):
		dizq = -dizq
	ds = (dizq+dder)/2
	dthe = (dder-dizq)/b
	cosen = math.cos((pos[2] + (dthe/2)))
	sen = math.sin((pos[2] + (dthe/2)))
	dp = np.array([ds*cosen , ds*sen,  dthe])
	pos = pos + dp
	fp = np.array([[1,0,-dp[1]],[0,1,dp[0]],[0,0,1]])
	sigs = np.array([[abs(dder),0],[0,abs(dizq)]])
	fs = np.array([[((cosen/2)-(ds*sen/(2*b))) , ((cosen/2)+(ds*sen/(2*b)))],[((sen/2)+(ds*cosen/(2*b))) , ((sen/2)-(ds*cosen/(2*b)))],[1/b , -1/b]])
	p1 = np.matmul( fp, cov )
	p2 = np.matmul( fs, sigs )
	cov = np.matmul( p1, fp.T ) + np.matmul( p2 , fs.T)

if __name__ == '__main__':
	global izqAnt, derAnt, derAct, izqAct, dirDer, dirIzq, pos, cov
	izqAct = 0
	derAct = 0
	dirDer = 0
	dirIzq = 0
	izqAnt = 0
	derAnt = 0
	start = [0,0,0]
	cov = np.zeros((3,3))
	rospy.init_node('odometria')
	rospy.wait_for_service('pos_inicio')
	met = rospy.ServiceProxy('pos_inicio', posInicio)
	req = met()
	pos = np.array(req.start)
	rospy.Subscriber('cuentasDer', Int32, callback)
	rospy.Subscriber('cuentasIzq', Int32, callback2)
	rospy.Subscriber('dirDer', Int32, callback3)
	rospy.Subscriber('dirIzq', Int32, callback4)
	tasa = rospy.Rate(200)
	while not rospy.is_shutdown():
		data = Covariance()
		data.sigma11 = cov[0][0]
		data.sigma12 = cov[0][1]
		data.sigma13 = cov[0][2]
		data.sigma21 = cov[1][0]
		data.sigma22 = cov[1][1]
		data.sigma23 = cov[1][2]
		data.sigma31 = cov[2][0]
		data.sigma32 = cov[2][1]
		data.sigma33 = cov[2][2]
		pub.publish(data)
		pub2.publish(data = pos.tolist())
		pose = Pose()
		pose.position.x = pos[0]
		pose.position.y = pos[1]
		pose.orientation.w = pos[2]
		pub1.publish(pose)
		tasa.sleep()
