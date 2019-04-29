#!/usr/bin/env python

# This controller extends the base DroneVideoDisplay class, adding a keypress handler to enable keyboard control of the drone

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
#import roslib; roslib.load_manifest('drone')
import rospy

# Load the DroneController class, which handles interactions with the drone, and the DroneVideoDisplay class, which handles video display
from drone_controller import BasicDroneController
from drone_video import DroneVideoDisplay
from std_msgs.msg import Float64MultiArray

# Finally the GUI libraries
from PySide import QtCore, QtGui
import numpy as np
from geometry_msgs.msg import Quaternion


def updateTrackingControl(pertX, pertXRate):
	#roll = limitCommRoll(pdCommandedRoll(pertX, pertXRate))
	roll = pdCommandedRoll(pertX, pertXRate)
	print(roll)
	#pitch = limitCommPitch(pdCommandedPitch(pertZ, pertZRate))
	controller.SetCommand(roll, pitch, yaw_velocity,z_velocity)

 #PD controller
def pdCommandedRoll(perturbedX, perturbedXRate):
	#simple continuous PD control law
	roll_c = perturbedX*rollGains[0,0] + perturbedXRate*rollGains[1,0]
	return roll_c

def DetBox(box):
	#print(box)
	xMin = box.y
	xMax = box.w
	if xMin < 0.3:
		pertX = 0.4 - xMin
	elif xMax > 0.7:
		pertX = 0.7 - xMax
	else:
		pertX = 0
	pertZ = 0
	pertXRate = 0
	pertZRate = 0
	updateTrackingControl(pertX, pertXRate)

# Setup the application
if __name__=='__main__':
	import sys
	# Firstly we setup a ros node, so that we can communicate with the other packages
	rospy.init_node('drone_PDcontroller')
	
	controller = BasicDroneController()

	rollGains = np.zeros((2,1))
	rollGains[1,0] = 1*2*(np.pi)*2/1.81 # f(2pi)*2/weight_kg
	rollGains[0,0] = (-1.8/4)*(rollGains[1,0])**2 # (-T/4)*Kd^2
	rollCommLimit = [-10, 10]

	pitch = 0
	roll = 0
	yaw_velocity = 0
	z_velocity = 0

	pertX = 0
	pertZ = 0
	pertXRate = 0
	pertZRate = 0

	subDetBox = rospy.Subscriber('/ardrone/pubDetBox',Quaternion,DetBox)
	rospy.spin()


	# and only progresses to here once the application has been shutdown
	#rospy.signal_shutdown('Great Flying!')

