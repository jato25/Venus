#!/usr/bin/python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Float32
from pylab import *
from pynput.keyboard import Key, Listener 
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import threading
import time
import sys 


robot = 2
r = 0.0925	##Radio de las ruedas del Pioneer obtenidas en el manual en m
l = 0.1900	##Distancia entre la rueda y el punto con el que se modela el robot en m
J1 = np.array([[1,0,l],[1,0,-l]]) 
inv_J2 = np.array([[1.0/r,0],[0,1.0/r]])
pub = rospy.Publisher('/motorsVel', Float32MultiArray, queue_size=10) #Se define que se publica en el topico de velocidad de las ruedas del motor
#Metodo que hace todas las suscripciones a los topicos y toma los valores entrados por parametro en la terminal

def arrancar():
	global xfin, yfin, thetafin
	#Suscripcion a topicos de ros
	rospy.init_node('punto2c', anonymous = True)
	rospy.Subscriber('/obstacles', Float32MultiArray ,obstacles)
	rospy.Subscriber('/pioneerPosition', Twist ,vecto)
	tasa = rospy.Rate(10)
	rospy.myargv(argv=sys.argv)
	try:
		xfin = float(sys.argv[1]) #Coordenada x entrada en la terminal
		yfin = float(sys.argv[2]) #Coordenada y entrada en la terminal
		thetafin = float(sys.argv[3]) #Angulo entrado en la terminal
	except:    								##Tratamiento de los argumentos ingresador por el usuario
		xfin = 40
		yfin = 40
		thetafin = math.pi/2
	time.sleep(1)
	#Se crean la cuadricula con las celdas para representar el mapa de V-rep
	crearCuadricula()
	threading.Thread(target=control).start()
	threading.Thread(target=plotPos).start()
	try:
		while not rospy.is_shutdown():
			tasa.sleep()
			publicar()
	except rospy.ServiceException as e:
		pass
#Metodo que obtiene las coordenadas de los obstaculos del topcio de ros	


def obstacles(data):	
	global obs
	obs = data.data
	
def publicar():
	global vec
	pub.publish(data = [vec[1],vec[0]])
#Metodo que crea la cuadricula que representa el mapa de V-rep

def R(theta): ##funcion que retorna la matriz de rotacion
	return np.array([[math.cos(theta),math.sin(theta),0],[-math.sin(theta),math.cos(theta),0],[0,0,1]])
#Metodo que busca el nodo en la matriz de nodo con las posciones que le entran por paramtero

def control():
	global posix, posiy, lastheta, vec, xfin, yfin, thetafin, bandera
	rho = 10
	beta = 20
	x_vec,y_vec = Astar()
	x_vec.reverse()
	y_vec.reverse()
	for i in range(len(x_vec)):
		while rho >= 0.08:
			dx = x_vec[i] - posix[-1]
			dy = y_vec[i] - posiy[-1]
			if (x_vec[i] == x_vec[-2] and y_vec[i] == y_vec[-2]):
				dtheta = lastheta - thetafin
			elif (i != len(x_vec)-1):
				dtheta = lastheta - math.atan2(y_vec[i+1]-y_vec[i],x_vec[i+1]-x_vec[i])
			else:
				dtheta = lastheta - thetafin
			rho = math.sqrt((dx)**2 + (dy)**2)
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
			kp = 0.6
			ka = 1.8
			v = kp * rho
			x = v*math.cos(lastheta)
			y = v*math.sin(lastheta)
			w = (ka*alpha) + (kb*(beta))
			vec = inv_J2.dot(J1.dot(R(lastheta).dot(np.array([x,y,w]))))
			time.sleep(0.2)
			if bandera:
				return False
		rho = 10
		beta = 20
	beta = 0.5
	while abs(beta) >= 0.01:
		kb = 0.3
		beta = -lastheta + thetafin
		if (beta >= 2*math.pi):
				beta = beta - 2*pi
		elif (beta <= -2*math.pi): 
			beta = beta + 2*pi
		w =(kb*(beta))
		x = 0
		y = 0
		vec = inv_J2.dot(J1.dot(R(lastheta).dot(np.array([x,y,w]))))
		time.sleep(0.2)
		if bandera:
			return False
	bandera = True
	vec = [0,0]
	return False

def ThreadInputs():
	with Listener(on_press = keypress) as listener:
		listener.join()

if __name__ == '__main__':	
	global obs, bandera, posix, posiy, lastheta, vec, xfin, yfin, thetafin, matriz, matNod, xplot, yplot
	obs = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
	posix = [0]
	posiy = [0]
	xplot = [0]
	yplot = [0]
	vec = [0,0]
	matriz = [[0  for i in range(200)]for j in range(200)]
	matNod = [[Nodo([i , j]) for i in range(200)]for j in range(200)]
	bandera = False
	xfin = 0
	yfin = 0
	lastheta = 0
	thetafin = 0
	try:
		threading.Thread(target=ThreadInputs).start()
		arrancar()
	except rospy.ServiceException:
		pass


