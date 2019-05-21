#!/usr/bin/python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Float32
from pylab import *
from pynput.keyboard import Key, Listener 
from geometry_msgs.msg import Twist
import numpy as np
import threading
import time
import sys

Pub = rospy.Publisher('/robotUncertainity', Covariance, queue_size=10)

r = 0.0925
l = 0.1900
J1_1 = np.array([[1,0,l],[1,0,-l],[0,1,0]])
inv_J1 = np.linalg.inv(J1_1)
J1 = np.array([[1,0,l],[1,0,-l]])
J2 = np.array([[r,0],[0,r],[0,0]])
inv_J2 = np.array([[1.0/r,0],[0,1.0/r]])
ka = 0
kb = 0
kp = 0.3
pub = rospy.Publisher('/motorsVel', Float32MultiArray, queue_size=10)

def arrancar():
	global nombreArchivo, vel_der, vel_izq, tiempo, xfin, yfin, thetafin
	rospy.init_node('punto3', anonymous = True)
	rospy.myargv(argv=sys.argv)
	try:
		xfin = float(sys.argv[1])
		yfin = float(sys.argv[2])
		thetafin = float(sys.argv[3])
	except:
		xfin = 4
		yfin = 4
		thetafin = math.pi/2
	rospy.Subscriber('/pioneerPosition', Twist ,vecto)
	rospy.Subscriber('/simulationTime', Float32, controlTiempo) 
	tasa = rospy.Rate(10)
	try:
		while not rospy.is_shutdown():
			tasa.sleep()		
	except rospy.ServiceException as e:
		pass
	
def vecto(data):
	global posix, posiy, lastheta, vec
	xact = data.linear.x
	yact = data.linear.y
	posix.append(xact)
	posiy.append(yact)
	lastheta = data.angular.z
	pub.publish(data = [vec[1],vec[0]]) 
	print(vec)

def ThreadInputs():
	with Listener(on_press = keypress) as listener:
		listener.join()

def keypress(key):
	global vel, bandera
	if key == Key.esc:
		bandera = True
		return False
	
	

	
def desplazamiento():
	global dsr, dsl, ds, dtheta, x_actual, y_acutal, theta_actual
	for i in range 
	
		dsl=(Nl*2*numpy.pi*32.5)/(442/34)
		dsr=(NR*2*numpy.pi*32.5)/(442/34)
		
		ds = (dsl+dsr)/2
		dtheta = (dsl-dsr)/l






def plotPos():
	global posix, posiy, bandera, posTeox, posTeoy, tiempoSimu, error, fin
	while not bandera:
		if fin == True:
			plt.clf()
			plt.plot(tiempoSimu, error)
			plt.ylabel('Magnitud del error')
			plt.xlabel('Tiempo de simulacion')
			plt.title('Comportamiento del error')
			plt.draw()
			plt.pause(0.8)
		else:
			plt.clf()
			plt.plot(posix,posiy,'p')
			plt.plot(posTeox,posTeoy)
			plt.ylabel('Posicion en y')
			plt.xlabel('Posicion en x')
			plt.title('Posicion del robot')
			plt.draw()
			plt.pause(0.8)
			plt.savefig('src/taller2_6/results/graficaPos3.png')
	if bandera == True:
		plt.savefig('src/taller2_6/results/graficaError3.png')
		plt.close()
		return False
		
		
def controlTiempo (data): 
	pass
'''	global posTeox, posTeoy, theta, ant, theta, tiempoSimu, error, fin
	if(ant == -1):
		tiempoSimu.append(data.data)
		zero = data.data
		ant = zero 
		errort = math.sqrt((posix[-1]-posTeox[-1])**2 + (posiy[-1]-posTeoy[-1])**2)
		error.append(errort)
	##elif(iterador < len(vector_tiempo)):
		##tiempoSimu.append(data.data)
		##errort = math.sqrt((posix[-1]-posTeox[-1])**2 + (posiy[-1]-posTeoy[-1])**2)*100
		##error.append(errort)
		##if (data.data - zero <= vector_tiempo[iterador]):
			###deltat = data.data - ant	
			###ant = data.data
			###vec = invR(theta).dot(inv_J1.dot(J2.dot(np.array([vector_izq[iterador],vector_der[iterador]]))))
		###	x = posTeox[-1] + vec[0]*deltat
		###	y = posTeoy[-1] + vec[1]*deltat
		##	theta = theta + vec[2]*deltat
		##	posTeox.append(x)
		##	posTeoy.append(y)
		##else:
			##zero = data.data
	else:
		pub.publish( data = [0, 0])
		fin = True'''
		
def R(theta):
	return np.array([[math.cos(theta),math.sin(theta),0],[-math.sin(theta),math.cos(theta),0],[0,0,1]])
	
def invR(theta):
	return np.array([[math.cos(theta),-math.sin(theta),0],[math.sin(theta),math.cos(theta),0],[0,0,1]])

def control():
	global posix, posiy, xfin, yfin, thetafin, lastheta, vec
	rho = math.sqrt((posix[-1]-xfin)**2 + (posiy[-1]-yfin)**2)
	beta = math.atan((posiy[-1]-yfin)/(posix[-1]-xfin)) 
	alpha = beta - lastheta
	v = kp * rho
	w = (ka+alpha) + (kb*beta)
	x = v*math.cos(lastheta)
	y = v*math.sin(lastheta)
	vec = inv_J2.dot(J1.dot(R(lastheta).dot(np.array([x,y,w]))))
	time.sleep(0.3)

def publicar():
    global vec
    Pub.publish(data = [vec[0],vec[1]])

if __name__ == '__main__':
	global bandera, posix, posiy, posTeox, posTeoy, lastheta, ant, tiempoSimu, error, fin, xfin, yfin, thetafin
	posTeox = []
	posTeoy = []
	posix = []
	posiy = []
	tiempoSimu = []
	error=[]
	lastheta = -math.pi
	posix.append(0)
	posiy.append(0)
	posTeox.append(0)
	posTeoy.append(0)
	ant = -1
	bandera = False
	fin = False
	xfin = 1
	yfin = 0
	thetafin = math.pi
	try:
		print("Presione una tecla (Esc para salir):")
		threading.Thread(target=ThreadInputs).start()
		threading.Thread(target=plotPos).start()
		threading.Thread(target=control).start()
		arrancar()
	except rospy.ServiceException:
		pass


