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
#Leds
GPIO.setup(9,GPIO.OUT)
GPIO.setup(10,GPIO.OUT)

der1 = GPIO.PWM(20, 100)
der2 = GPIO.PWM(21, 100)
izq1 = GPIO.PWM(12, 100)
izq2 = GPIO.PWM(16, 100)
der1.start(0)
der2.start(0)
izq1.start(0)
izq2.start(0)
velocidadD = 0
errorAntD = 0
promedioD = 0
tiempoAntD = time.time()
velocidadI = 0
errorAntI = 0
promedioI = 0
tiempoAntI = time.time()
pub = rospy.Publisher('direcciones', Int32MultiArray , queue_size=10)
kp = 5.8
ki = 4
kd = 0.08

def controlD():
	global velocidadD, integralD, tiempoAntD, promedioD, errorAntD, velD, dirD, ciclDer
	error = abs(velD) - velocidadD
	integralD += error*(time.time()-tiempoAntD)
	derivada = (velocidadD - errorAntD)/(time.time()-tiempoAntD)
	errorAntD = velocidadD
	tiempoAntD = time.time()
	#if (vel >= 3):
	ciclo = error*kp + ki*integralI - kd*derivada
	'''else:
		ciclo = error*0.08 + 1*integralD - 0.08*derivada'''
	if(abs(velD) <= 0.06):
		ciclo = 0
	if (ciclo > 100):
		ciclo = 100
	elif(ciclo < 0):
		ciclo = 0
	if (velD > 0):
		dirD = 1
		ciclDer=ciclo
		der1.ChangeDutyCycle(ciclo)
		der2.ChangeDutyCycle(0)
	else:
		dirD = 0
		der2.ChangeDutyCycle(ciclo)
		der1.ChangeDutyCycle(0)
	

def controlI():
	# velocidad es la del encoder, vel es la del controlador 
	global velocidadI, integralI, tiempoAntI, promedioI, errorAntI, velI, dirI,ciclIzq
	error = abs(velI) - velocidadI
	integralI += error*(time.time()-tiempoAntI)
	derivada = (velocidadI - errorAntI)/(time.time()-tiempoAntI)
	errorAntI = velocidadI
	tiempoAntI = time.time()
	#if (vel >= 3):
	ciclo = error*kp + ki*integralI - kd*derivada
	'''else:
		ciclo = error*0.08 + 1*integralI - 0.08*derivada'''
	if(abs(velI) <= 0.06):
		ciclo = 0 
	if (ciclo > 100):
		ciclo = 100
	elif(ciclo < 0):
		ciclo = 0
	if (velI > 0):
		dirI = 1
		ciclIzq=ciclo
		izq1.ChangeDutyCycle(ciclo)
		izq2.ChangeDutyCycle(0)
	else:
		dirI = 0
		izq2.ChangeDutyCycle(ciclo)
		izq1.ChangeDutyCycle(0)
		
	
def callback(data):
	global velocidadD, velocidadI
	velocidadD = data.data[0]
	velocidadI = data.data[1]


def callback3(data):
	global velD, velI
	velD = data.data[0]
	velI = data.data[1]
	velD = 3
	velI = 3

def direccionales():
	global ciclIzq, ciclDer, lIzq, lDer
	if(ciclIzq>ciclDer):
		GPIO.output(9, True)
		GPIO.output(10, False)
	elif(ciclIzq<ciclDer):
		GPIO.output(9, False)
		GPIO.output(10, True)
	else:
		GPIO.output(9, False)
		GPIO.output(10, False)
	



if __name__ == '__main__':
	global velD, velI, dirD, dirI, integralD, integralI, ciclDer , ciclIzq
	ciclDer=0
	ciclIzq=0
	dirD = 0
	dirI = 0
	velD = 0
	velI = 0
	integralD = 0
	integralI = 0
	rospy.init_node('controladorEnconder')
	rospy.Subscriber('motorsVel', Float32MultiArray, callback)
	rospy.Subscriber('velocidades', Float32MultiArray, callback3)
	tasa = rospy.Rate(100)
	timeInt = time.time()
	while not rospy.is_shutdown():
		controlD()
		controlI()
		if time.time() - timeInt > 0.5:
			integralD = 0
			integralI = 0
			timeInt = time.time()
		pub.publish(data = [dirD , dirI])	
		#direccionales()
		tasa.sleep()
	GPIO.cleanup()
