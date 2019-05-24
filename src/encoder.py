#!/usr/bin/python
import RPi.GPIO as GPIO
import os, rospy, time
import numpy as np
from std_msgs.msg import *

#Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(2,GPIO.IN)
GPIO.setup(3,GPIO.IN)
GPIO.setup(4,GPIO.IN)
GPIO.setup(17,GPIO.IN)
pub = rospy.Publisher('motorsVel', Float32MultiArray, queue_size=10)

#Callbacks
def CuentaA(channel):
	global contaA, tiempoAnt1, velocidad1, promedio1
	contaA += 1
	if (contaA % 40 == 0):	
		deltaT = time.time() - tiempoAnt1
		velocidad1 = [2*40*np.pi/(442*deltaT)] + velocidad1[0:-1]
		promedio1 = np.mean(velocidad1)
		tiempoAnt1 = time.time()

def CuentaB(channel):
	global contaB, tiempoAnt2, velocidad2, promedio2
	contaB += 1
	if (contaB % 40 == 0):
		deltaT = time.time() - tiempoAnt2
		velocidad2 = [2*40*np.pi/(442*deltaT)] + velocidad2[0:-1]
		promedio2 = np.mean(velocidad2)
		tiempoAnt2 = time.time()

def CuentaC(channel):
	global contaC, tiempoAnt3, velocidad3, promedio3
	contaC += 1
	if (contaC % 40 == 0):
		deltaT = time.time() - tiempoAnt3
		velocidad3 = [2*40*np.pi/(442*deltaT)] + velocidad3[0:-1]
		promedio3 = np.mean(velocidad3)
		tiempoAnt3 = time.time()

def CuentaD(channel):
	global contaD, tiempoAnt4, velocidad4, promedio4
	contaD += 1
	if (contaD % 40 == 0):
		deltaT = time.time() - tiempoAnt4
		velocidad4 = [2*40*np.pi/(442*deltaT)] + velocidad4[0:-1]
		promedio4 = np.mean(velocidad4)
		tiempoAnt4 = time.time()

def resetD():
	global tiempoAnt1, velocidad1, tiempoAnt2, velocidad2, promedio1, promedio2
	promedio1 = 0 
	promedio2 = 0
	tiempoAnt1 = time.time()
	tiempoAnt2 = time.time()
	velocidad1 = np.zeros(30).tolist()
	velocidad2 = np.zeros(30).tolist()

def resetI():
	global tiempoAnt3, velocidad3, tiempoAnt4, velocidad4, promedio3, promedio4
	promedio3 = 0 
	promedio4 = 0
	tiempoAnt3 = time.time()
	tiempoAnt4 = time.time()
	velocidad3 = np.zeros(30).tolist()
	velocidad4 = np.zeros(30).tolist()

if __name__ == '__main__':
	global contaA, contaB, promedio1, promedio2, contaC, contaD, promedio3, promedio4, contaApre,contaBpre,contaCpre,contaDpre
	rospy.init_node('encoder')
	canalB = False
	canalD = False
	direcD = False
	direcI = False
	promedio1 = 0 
	promedio2 = 0
	promedio3 = 0 
	promedio4 = 0
	tiempoAnt1 = time.time()
	tiempoAnt2 = time.time()
	tiempoAnt3 = time.time()
	tiempoAnt4 = time.time()	
	contaA = 0 
	contaB = 0
	contaC = 0 
	contaD = 0
	velocidad1 = np.zeros(30).tolist()
	velocidad2 = np.zeros(30).tolist()
	velocidad3 = np.zeros(30).tolist()
	velocidad4 = np.zeros(30).tolist()
	GPIO.add_event_detect(2, GPIO.RISING, callback = CuentaA)
	GPIO.add_event_detect(3, GPIO.RISING, callback = CuentaB)
	GPIO.add_event_detect(4, GPIO.RISING, callback = CuentaC)
	GPIO.add_event_detect(17, GPIO.RISING, callback = CuentaD)
	tasa = rospy.Rate(100)
	contaApre = 0
	contaBpre = 0
	contaCpre = 0
	contaDpre = 0
	while not rospy.is_shutdown():
		if contaA == contaApre and contaB == contaBpre:
			resetD()
		if contaC == contaCpre and contaD == contaDpre:
			resetI()
		contaApre = contaA
		contaBpre = contaB
		contaCpre = contaC
		contaDpre = contaD
		pub.publish(data = [(promedio1 + promedio2)/2 , (promedio3 + promedio4)/2])
		tasa.sleep()
	GPIO.cleanup()
