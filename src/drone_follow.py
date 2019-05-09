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
from geometry_msgs.msg import Twist

# Here we define the keyboard map for our controller (note that python has no enums, so we use a class)
class KeyMapping(object):
	PitchForward     = QtCore.Qt.Key.Key_E
	PitchBackward    = QtCore.Qt.Key.Key_D
	RollLeft         = QtCore.Qt.Key.Key_S
	RollRight        = QtCore.Qt.Key.Key_F
	YawLeft          = QtCore.Qt.Key.Key_W
	YawRight         = QtCore.Qt.Key.Key_R
	IncreaseAltitude = QtCore.Qt.Key.Key_Q
	DecreaseAltitude = QtCore.Qt.Key.Key_A
	Takeoff          = QtCore.Qt.Key.Key_Y
	Land             = QtCore.Qt.Key.Key_H
	Emergency        = QtCore.Qt.Key.Key_Space

# Our controller definition, note that we extend the DroneVideoDisplay class
class KeyboardController(DroneVideoDisplay):
	def __init__(self):
		super(KeyboardController,self).__init__()
		
		self.rollGains = np.zeros((2,1))
		self.rollGains[1,0] = 1*2*(np.pi)*2/1.81 # f(2pi)*2/weight_kg
		self.rollGains[0,0] = (1)*(-1.8/4)*(self.rollGains[1,0])**2 # (-T/4)*Kd^2
		self.rollCommLimit = [-10, 10]
		self.pitch = 0
		self.roll = 0
		self.yaw_velocity = 0
		self.z_velocity = 0

		self.pertX = 0
		self.pertZ = 0
		self.pertXRate = 0
		self.pertZRate = 0

		self.subDetBox = rospy.Subscriber('/ardrone/pubDetBox',Twist,self.DetBox_fun)

	def updateTrackingControl(self):
		#self.roll = self.limitCommRoll(self.pdCommandedRoll(pertX, pertXRate))
		rollCmd = self.pdCommandedRoll(self.pertX, self.pertXRate)
		max_s = +1
		min_s = -1
		max_i = +8
		min_i = -8
		self.roll = (max_s - min_s)*(rollCmd - min_i)/(max_i - min_i) + min_s
		if self.roll > 0:
			print("Roll Right: " + str(self.roll))
		elif self.roll < 0:
			print("Roll Left: " + str(self.roll))
		else:
			print("Don't Move")
		#self.pitch = self.limitCommPitch(self.pdCommandedPitch(pertZ, pertZRate))
		controller.SetCommand(-self.roll, 0, 0, 0)

	 #PD controller
	def pdCommandedRoll(self, perturbedX, perturbedXRate):
		#simple continuous PD control law
		roll_c = perturbedX*self.rollGains[0,0] + perturbedXRate*self.rollGains[1,0]
		return roll_c

	def DetBox_fun(self, box):
		#print(box)
		xMin = box.linear.y
		xMax = box.angular.x
		center_pt = (xMin + xMax)/2
		'''if xMin < 0.3:
			self.pertX = 0.4 - xMin
		elif xMax > 0.7:
			self.pertX = 0.7 - xMax
		else:
			self.pertX = 0'''
		if center_pt > 0.6 or center_pt < 0.4:
			self.pertX = 0.5 - center_pt
		else:
			self.pertX = 0
		print("Box Center Location: " + str(center_pt*100) + " %")
		self.pertZ = 0
		self.pertXRate = 0
		self.pertZRate = 0
		self.updateTrackingControl()

# We add a keyboard handler to the DroneVideoDisplay to react to keypresses
	def keyPressEvent(self, event):
		key = event.key()

		# If we have constructed the drone controller and the key is not generated from an auto-repeating key
		if controller is not None and not event.isAutoRepeat():
			# Handle the important cases first!
			if key == KeyMapping.Emergency:
				controller.SendEmergency()
			elif key == KeyMapping.Takeoff:
				controller.SendTakeoff()
			elif key == KeyMapping.Land:
				controller.SendLand()
			else:
				# Now we handle moving, notice that this section is the opposite (+=) of the keyrelease section
				if key == KeyMapping.YawLeft:
					self.yaw_velocity += 1
				elif key == KeyMapping.YawRight:
					self.yaw_velocity += -1

				elif key == KeyMapping.PitchForward:
					self.pitch += 1
					
				elif key == KeyMapping.PitchBackward:
					self.pitch += -1

				elif key == KeyMapping.RollLeft:
					self.roll += 1
				elif key == KeyMapping.RollRight:
					self.roll += -1

				elif key == KeyMapping.IncreaseAltitude:
					self.z_velocity += 1
				elif key == KeyMapping.DecreaseAltitude:
					self.z_velocity += -1
				print(self.pitch)
				print(self.roll)
			# finally we set the command to be sent. The controller handles sending this at regular intervals
			controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)


	def keyReleaseEvent(self,event):
		key = event.key()

		# If we have constructed the drone controller and the key is not generated from an auto-repeating key
		if controller is not None and not event.isAutoRepeat():
			# Note that we don't handle the release of emergency/takeoff/landing keys here, there is no need.
			# Now we handle moving, notice that this section is the opposite (-=) of the keypress section
			if key == KeyMapping.YawLeft:
				self.yaw_velocity -= 1
			elif key == KeyMapping.YawRight:
				self.yaw_velocity -= -1

			elif key == KeyMapping.PitchForward:
				self.pitch -= 1
			elif key == KeyMapping.PitchBackward:
				self.pitch -= -1

			elif key == KeyMapping.RollLeft:
				self.roll -= 1
			elif key == KeyMapping.RollRight:
				self.roll -= -1

			elif key == KeyMapping.IncreaseAltitude:
				self.z_velocity -= 1
			elif key == KeyMapping.DecreaseAltitude:
				self.z_velocity -= -1

			# finally we set the command to be sent. The controller handles sending this at regular intervals
			controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)



# Setup the application
if __name__=='__main__':
	import sys
	# Firstly we setup a ros node, so that we can communicate with the other packages
	rospy.init_node('drone_follow')

	# Now we construct our Qt Application and associated controllers and windows
	app = QtGui.QApplication(sys.argv)
	controller = BasicDroneController()
	display = KeyboardController()

	display.show()

	# executes the QT application
	status = app.exec_()

	# and only progresses to here once the application has been shutdown
	rospy.signal_shutdown('Great Flying!')
	sys.exit(status)
