#!/usr/bin/python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Float32
from pylab import *
from pynput.keyboard import Key, Listener 
from geometry_msgs.msg import Twist
#import matplotlib.pyplot as plt
import threading
import time
import sys

Pub = rospy.Publisher('/robotUncertainity', Covariance, queue_size=10)

def arrancar():
  rospy.init_node('ControlPos', anonymous = True)
  tasa = rospy.Rate(10)
  rospy.myargv(argv=sys.argv)
  try:
		while not rospy.is_shutdown():
			tasa.sleep()
	except rospy.ServiceException as e:
		pass
def publicar():
    global vec
    Pub.publish(data = [vec[0],vec[1]])
    
if __name__ == '__main__':	
  try:
		arrancar()
	except rospy.ServiceException:
		pass
