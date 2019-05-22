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
der1 = GPIO.PWM(20, 100)
der2 = GPIO.PWM(21, 100)
izq1 = GPIO.PWM(12, 100)
izq2 = GPIO.PWM(16, 100)
der1.start(0)
der2.start(0)
izq1.start(0)
izq2.start(0)
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
pub5 = rospy.Publisher('dirDer', Int32, queue_size=10)
pub6 = rospy.Publisher('dirIzq', Int32, queue_size=10)
kp = 0.8
ki = 0.1
kd = 0.1

def controlD():
	global velocidadD, integralD, tiempoAntD, promedioD, errorAntD, velD, dirD
	error = abs(velD) - velocidadD
	integralD += error*(time.time()-tiempoAntD)
	derivada = (velocidadD - errorAntD)/(time.time()-tiempoAntD)
	errorAntD = velocidadD
	tiempoAntD = time.time()
	#if (vel >= 3):
	ciclo = error*kp + ki*integralI - kd*derivada
	'''else:
		ciclo = error*0.08 + 1*integralD - 0.08*derivada'''
	if(abs(velD) <= 0.03):
		ciclo = 0
	if (ciclo > 100):
		ciclo = 100
	elif(ciclo < 0):
		ciclo = 0
	if (velD > 0):
		dirD = 1
		der1.ChangeDutyCycle(ciclo)
		der2.ChangeDutyCycle(0)
	else:
		dirD = 0
		der2.ChangeDutyCycle(ciclo)
		der1.ChangeDutyCycle(0)

def controlI():
	# velocidad es la del encoder, vel es la del controlador 
	global velocidadI, integralI, tiempoAntI, promedioI, errorAntI, velI, dirI
	error = abs(velI) - velocidadI
	integralI += error*(time.time()-tiempoAntI)
	derivada = (velocidadI - errorAntI)/(time.time()-tiempoAntI)
	errorAntI = velocidadI
	tiempoAntI = time.time()
	#if (vel >= 3):
	ciclo = error*kp + ki*integralI - kd*derivada
	'''else:
		ciclo = error*0.08 + 1*integralI - 0.08*derivada'''
	if(abs(velI) <= 0.03):
		ciclo = 0 
	if (ciclo > 100):
		ciclo = 100
	elif(ciclo < 0):
		ciclo = 0
	if (velI > 0):
		dirI = 1
		izq1.ChangeDutyCycle(ciclo)
		izq2.ChangeDutyCycle(0)
	else:
		dirI = 0
		izq2.ChangeDutyCycle(ciclo)
		izq1.ChangeDutyCycle(0)
		
	
def callback(data):
	global velocidadD
	velocidadD = data.data

def callback2(data):
	global velocidadI
	velocidadI = data.data

def callback3(data):
	global velD, velI
	velD = data.data[0]
	velI = data.data[1]

if __name__ == '__main__':
	global velD, velI, dirD, dirI
	dirD = 0
	dirI = 0
	velD = 0
	velI = 0
	rospy.init_node('controladorEnconder')
	rospy.Subscriber('encoderDer', Float32, callback)
	rospy.Subscriber('encoderIzq', Float32, callback2)
	rospy.Subscriber('velocidades', Float32MultiArray, callback3)
	tasa = rospy.Rate(500)
	while not rospy.is_shutdown():
		controlD()
		controlI()
		pub6.publish(dirI)
		pub5.publish(dirD)	
		tasa.sleep()
	GPIO.cleanup()
