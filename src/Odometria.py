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

r = 32.5
l = 90

def arrancar():
	global nombreArchivo, vel_der, vel_izq, tiempo, xfin, yfin, thetafin
	rospy.init_node('Odometria', anonymous = True)
	rospy.myargv(argv=sys.argv)
	try:
		xfin = float(sys.argv[1])
		yfin = float(sys.argv[2])
		thetafin = float(sys.argv[3])
	except:
		xfin = 4
		yfin = 4
		thetafin = math.pi/2
		
	rospy.Subscriber('cuentasDer', Int32 ,cuentasderecha)
	rospy.Subscriber('cuentasIzq', Int32,cuentasizquierda) 
	tasa = rospy.Rate(10)
	
	try:
		while not rospy.is_shutdown():
			tasa.sleep()		
	except rospy.ServiceException as e:
		pass
	
	

def ThreadInputs():
	with Listener(on_press = keypress) as listener:
		listener.join()

def keypress(key):
	global vel, bandera
	if key == Key.esc:
		bandera = True
		return False

	
def cuentasderecha(data):	
	global NR
	NR.data
	
def cuentasizquierda(data):
	global Nl
	Nl.data
	
	

# calculo de desplazamiento del robot	
def desplazamiento():
	global dsr, dsl, ds, dtheta, NR, Nl
	for i in range 
	
		dsl=(Nl*2*numpy.pi*32.5)/(442/34)
		dsr=(NR*2*numpy.pi*32.5)/(442/34)
		
		ds = (dsl+dsr)/2
		dtheta = (dsl-dsr)/l

		np.array([[ds*math.cos(theta+dtheta/2)],[ds*math.sin(theta+dtheta/2)],[dtheta]])




	
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


