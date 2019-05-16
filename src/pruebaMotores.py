#!/usr/bin/python
import sys
import rospy
import time
import RPi.GPIO as GPIO

Forward=16
Backward=26

mode=GPIO.getmode()
GPIO.setmode(GPIO.BCM)
GPIO.setup(Forward, GPIO.OUT)

GPIO.setup(Backward, GPIO.IN)


def forward(x):
	GPIO.output(Forward, 1)
	print("Moving Forward")
	time.sleep(x)
	GPIO.output(Forward,0)

def reverse(x):
	GPIO.output(Backward, GPIO.HIGH)
	print("Moving Backward")
	time.sleep(x)
	GPIO.output(Backward, GPIO.LOW)

if __name__ == '__main__':
	#rospy.init_node('roberta' , anonymous = True)
	print('Hola Javi')
	while True:
		if (GPIO.input(26)== 1)
			

				print(GPIO.input(26))
		#time.sleep(0.5)
	##forward(30)
	##GPIO.cleanup()

