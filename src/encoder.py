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
pub1 = rospy.Publisher('encoderDer', Float32, queue_size=10)
pub2 = rospy.Publisher('cuentasDer', Int32, queue_size=10)
pub3 = rospy.Publisher('encoderIzq', Float32, queue_size=10)
pub4 = rospy.Publisher('cuentasIzq', Int32, queue_size=10)

#Callbacks
def CuentaA(channel):
	global contaA, tiempoAnt1, velocidad1, promedio1
	contaA += 1
	deltaT = time.time() - tiempoAnt1
	velocidad1 = [2*np.pi/(2*442*deltaT)] + velocidad1[0:-1]
	promedio1 = np.mean(np.array(velocidad1))
	tiempoAnt1 = time.time()

def CuentaB(channel):
	global contaB, tiempoAnt2, velocidad2, promedio2
	contaB += 1
	deltaT = time.time() - tiempoAnt2
	velocidad2 = [2*np.pi/(2*442*deltaT)] + velocidad2[0:-1]
	promedio2 = np.mean(np.array(velocidad2))
	tiempoAnt2 = time.time()

def CuentaC(channel):
	global contaC, tiempoAnt3, velocidad3, promedio3
	contaC += 1
	deltaT = time.time() - tiempoAnt3
	velocidad3 = [2*np.pi/(2*442*deltaT)] + velocidad3[0:-1]
	promedio3 = np.mean(velocidad3)
	tiempoAnt3 = time.time()

def CuentaD(channel):
	global contaD, tiempoAnt4, velocidad4, promedio4
	contaD += 1
	deltaT = time.time() - tiempoAnt4
	velocidad4 = [2*np.pi/(2*442*deltaT)] + velocidad4[0:-1]
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
	pub1.publish((promedio1 + promedio2)/2)

def resetI():
	global tiempoAnt3, velocidad3, tiempoAnt4, velocidad4, promedio3, promedio4
	promedio3 = 0 
	promedio4 = 0
	tiempoAnt3 = time.time()
	tiempoAnt4 = time.time()
	velocidad3 = np.zeros(30).tolist()
	velocidad4 = np.zeros(30).tolist()
	pub3.publish((promedio3 + promedio4)/2)

if __name__ == '__main__':
	global contaA, contaB, promedio1, promedio2, contaC, contaD, promedio3, promedio4
	rospy.init_node('encoder')
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
	GPIO.add_event_detect(3, GPIO.BOTH, callback = CuentaA)
	GPIO.add_event_detect(2, GPIO.BOTH, callback = CuentaB)
	GPIO.add_event_detect(4, GPIO.BOTH, callback = CuentaC)
	GPIO.add_event_detect(17, GPIO.BOTH, callback = CuentaD)
	tasa = rospy.Rate(110)
	contaApre = 0
	contaBpre = 0
	contaCpre = 0
	contaDpre = 0
	tiempoCuentas = time.time()
	while not rospy.is_shutdown():
		pub1.publish((promedio1 + promedio2)/2)
		pub2.publish((contaA + contaB)/2)
		pub3.publish((promedio3 + promedio4)/2)
		pub4.publish((contaC + contaD)/2)
		tasa.sleep()
		if time.time() - tiempoCuentas > 0.04:
			if contaA == contaApre and contaB == contaBpre:
				resetD()
			if contaC == contaCpre and contaD == contaDpre:
				resetI()
			contaApre = contaA
			contaBpre = contaB
			contaCpre = contaC
			contaDpre = contaD
	GPIO.cleanup()
