import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

TRIGGER1 = 21
ECHO1 = 20
TRIGGER2 = 26
ECHO2 = 19
TRIGGER3 = 13
ECHO3 = 6

GPIO.setup(TRIGGER1, GPIO.OUT)
GPIO.setup(ECHO1, GPIO.IN)
GPIO.setup(TRIGGER2 ,GPIO.OUT)
GPIO.setup(ECHO2, GPIO.IN)
GPIO.setup(TRIGGER3, GPIO.OUT)
GPIO.setup(ECHO3, GPIO.IN)

def distancia1():
	GPIO.output(TRIGGER1, True)
	time.sleep(0.00002)
	GPIO.output(TRIGGER1, False)
	inicio = time.clock()
	fin = time.clock()
	while(GPIO.input(ECHO1) == 0):
		inicio = time.clock()
	while(GPIO.input(ECHO1) ==  1):
		fin = time.clock()
	deltaT = fin - inicio
	distance = (deltaT * 34300)/2.0
	return distance

def distancia2():
	GPIO.output(TRIGGER2, True)
	time.sleep(0.00002)
	GPIO.output(TRIGGER2, False)
	inicio = time.clock()
	fin = time.clock()
	while(GPIO.input(ECHO2) == 0):
		inicio = time.clock()
	while(GPIO.input(ECHO2) ==  1):
		fin = time.clock()
	deltaT = fin - inicio
	distance = (deltaT * 34300)/2.0
	return distance

def distancia3():
	GPIO.output(TRIGGER3, True)
	time.sleep(0.00002)
	GPIO.output(TRIGGER3, False)
	inicio = time.clock()
	fin = time.clock()
	while(GPIO.input(ECHO3) == 0):
		inicio = time.clock()
	while(GPIO.input(ECHO3) ==  1):
		fin = time.clock()
	deltaT = fin - inicio
	distance = (deltaT * 34300)/2.0
	return distance



if __name__ == '__main__':
	while True:
		dist = distancia1()
		print(dist)
		#dist = distancia2()
		#print(dist)
		#dist = distancia3()
		#print(dist)
		time.sleep(0.5) 
	GPIO.cleanup()

