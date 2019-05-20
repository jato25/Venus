#!/usr/bin/python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Float32
from pylab import *
from pynput.keyboard import Key, Listener 
from geometry_msgs.msg import Twist
#import matplotlib.pyplot as plt
import threading
import time
import sys 

pub1 = rospy.Publisher('Velocidades', Float32MultiArray, queue_size=10)


robot = 170
r = 32.5	##Radio de las ruedas del Pioneer obtenidas en el manual en mm
l = 90.0	##Distancia entre la rueda y el punto con el que se modela el robot en mm
J1 = np.array([[1,0,l],[1,0,-l]]) 
inv_J2 = np.array([[1.0/r,0],[0,1.0/r]])

#Metodo que hace todas las suscripciones a los topicos y toma los valores entrados por parametro en la terminal
def arrancar():
	#Suscripcion a topicos de ros
	rospy.init_node('ControlPos', anonymous = True)
	tasa = rospy.Rate(10)
	rospy.myargv(argv=sys.argv)
	rospy.Suscriber('PosRobotSiguiente', Float32MultiArray, LeerPosSiguiente)
	rospy.Suscriber('PosRobot', Float32MultiArray, LeerPos)
	#Se crean la cuadricula con las celdas para representar el mapa de V-rep
	try:
		while not rospy.is_shutdown():
			tasa.sleep()
			control()
			publicar()
	except rospy.ServiceException as e:
		pass
#Metodo que obtiene las coordenadas de los obstaculos del topcio de ros	


def publicar():
	global vec
	pub1.publish(data = [vec[0],vec[1]])
#Metodo que crea la cuadricula que representa el mapa de V-rep
def LeerPosSiguiente(data):
	global posix, posiy, thetafin
	posix=data.data[0]
	posiy=data.data[1]
	thetafin=data.data[2]
def LeerPos(data):
	global x_vec, y_vec, lastheta
	x_vec= data.data[0]
	y_vec= data.data[1]
	lastheta=data.data[2]

def R(theta): ##funcion que retorna la matriz de rotacion
	return np.array([[math.cos(theta),math.sin(theta),0],[-math.sin(theta),math.cos(theta),0],[0,0,1]])
#Metodo que busca el nodo en la matriz de nodo con las posciones que le entran por paramtero

def control():
	global posix, posiy, lastheta, vec, bandera, x_vec, y_vec

	#Tomar las posiciones del topico de 
	#x_vec,y_vec = Astar()

	dx = x_vec - posix
	dy = y_vec - posiy
	dtheta = lastheta - thetafin
	rho=1.08
	#rho = math.sqrt((dx)**2 + (dy)**2)
	alpha = -lastheta + math.atan2(dy,dx)	#Se calculan los errores en coordenadas esfericas y se calcula la velocidad de acuerdo con kp
	beta = -math.atan2(dy,dx) - dtheta
	if (alpha >= 2*math.pi):
		alpha = alpha - 2*pi
	elif (alpha <= -2*math.pi):
		alpha = alpha + 2*pi
	if (beta >= 2*math.pi):
		beta = beta - 2*pi
	elif (beta <= -2*math.pi):
		beta = beta + 2*pi
	#Se definen las constantes de control proporcional por sintonizacion
	kb = 0.07
	kp =1
	ka = 1.8
	v = kp * rho
	x = v*math.cos(lastheta)
	y = v*math.sin(lastheta)
	w = (ka*alpha) + (kb*(beta))
	vec = inv_J2.dot(J1.dot(R(lastheta).dot(np.array([x,y,w]))))
		
if __name__ == '__main__':	
	global posix, posiy, lastheta, vec, x_vec, y_vec, thetafin
	posix = [0]
	posiy = [0]
	vec = [0,0]
	#matriz = [[0  for i in range(200)]for j in range(200)]
	#matNod = [[Nodo([i , j]) for i in range(200)]for j in range(200)]
	x_vec = 0
	y_vec = 0
	lastheta = 0
	thetafin = 0
	try:
		arrancar()
	except rospy.ServiceException:
		pass
