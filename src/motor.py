#!/usr/bin/python
import sys
import time
import RPi.GPIO as GPIO

GPIO.cleanup()
 Forward=26
 Backward=20
 sleeptime=1

GPIO.setmode(GPIO.BCM)
GPIO.setup(Forward, GPIO.OUT)
GPIO.setup(Backward, GPIO.OUT)

def forward(x):
 GPIO.output(Forward, GPIO.TRUE)
 print("Moving Forward")
 time.sleep(x)
 GPIO.output(Forward, GPIO.FALSE)

def reverse(x):
 GPIO.output(Backward, GPIO.TRUE)
 print("Moving Backward")
 time.sleep(x)
 GPIO.output(Backward, GPIO.FALSE)

while (1):

forward(5)

	reverse(5)

	GPIO.cleanup()
