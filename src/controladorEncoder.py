#!/usr/bin/python
import RPi.GPIO as GPIO
import os, rospy, time
import numpy as np
from std_msgs.msg import *

GPIO.setmode(GPIO.BCM)
GPIO.setup(21,GPIO.OUT)
GPIO.setup(20,GPIO.OUT)
GPIO.setup(16,GPIO.OUT)
GPIO.setup(12,GPIO.OUT)
der1 = GPIO.PWM(21, 100)
der2 = GPIO.PWM(20, 100)
izq1 = GPIO.PWM(16, 100)
izq2 = GPIO.PWM(12, 100)
der1.start(0)
der2.start(0)
izq1.start(0)
izq2.start(0)
VEL1 = 1
VEL2 = 0.5
integralD = 0
velocidadD = 0
errorAntD = 0
promedioD = 0
tiempoAntD = time.time()
integralI = 0
velocidadI = 0
errorAntI = 0
promedioI = 0
tiempoAntI = time.time()

def controlD(vel):
	global velocidadD, integralD, tiempoAntD, promedioD, errorAntD
	error = abs(vel) - velocidadD
	integralD += error*(time.time()-tiempoAntD)
	derivada = (velocidadD - errorAntD)/(time.time()-tiempoAntD)
	errorAntD = velocidadD
	tiempoAntD = time.time()
	#if (vel >= 3):
	ciclo = error*0.4 + 0.7*integralD - 0.1*derivada
	'''else:
		ciclo = error*0.08 + 1*integralD - 0.08*derivada'''
	if(vel == 0):
		ciclo = 0 
	if (ciclo >100):
		ciclo=100
	elif(ciclo<0):
		ciclo = 0
	if (vel > 0):
		der1.ChangeDutyCycle(ciclo)
		der2.ChangeDutyCycle(0)
	else:
		der2.ChangeDutyCycle(ciclo)
		der1.ChangeDutyCycle(0)

def controlI(vel):
	global velocidadI, integralI, tiempoAntI, promedioI, errorAntI
	error = abs(vel) - velocidadI
	integralI += error*(time.time()-tiempoAntI)
	derivada = (velocidadI - errorAntI)/(time.time()-tiempoAntI)
	errorAntI = velocidadI
	tiempoAntI = time.time()
	#if (vel >= 3):
	ciclo = error*0.5 + 0.7*integralI - 0.1*derivada
	'''else:
		ciclo = error*0.08 + 1*integralI - 0.08*derivada'''
	if(vel == 0):
		ciclo = 0 
	if (ciclo >100):
		ciclo=100
	elif(ciclo<0):
		ciclo = 0
	if (vel > 0):
		izq1.ChangeDutyCycle(ciclo)
		izq2.ChangeDutyCycle(0)
	else:
		izq2.ChangeDutyCycle(ciclo)
		izq1.ChangeDutyCycle(0)
		
	
def callback(data):
	global velocidadD
	velocidadD = data.data

def callback2(data):
	global velocidadI
	velocidadI = data.data

if __name__ == '__main__':
	rospy.init_node('controladorEnconder')
	rospy.Subscriber('encoderDer', Float32, callback)
	rospy.Subscriber('encoderIzq', Float32, callback2)
	tasa = rospy.Rate(110)
	while not rospy.is_shutdown():
		controlD(VEL2)
		controlI(VEL1)
		tasa.sleep()
	GPIO.cleanup()
