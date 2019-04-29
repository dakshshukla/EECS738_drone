#!/usr/bin/env python

# A basic video display window for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials_getting_started

# This display window listens to the drone's video feeds and updates the display at regular intervals
# It also tracks the drone's status and any connection problems, displaying them in the window's status bar
# By default it includes no control functionality. The class can be extended to implement key or mouse listeners if required

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
#import roslib; roslib.load_manifest('drone')
import rospy

# Import the two types of messages we're interested in
from sensor_msgs.msg import Image    	 # for receiving the video feed
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback

# We need to use resource locking to handle synchronization between GUI thread and ROS topic callbacks
from threading import Lock

# An enumeration of Drone Statuses
from drone_status import DroneStatus

# The GUI libraries
from PySide import QtCore, QtGui
from cv_bridge import CvBridge

import numpy as np
import tensorflow as tf
import time

import cv2
import rospkg
from object_detection.utils import label_map_util
from object_detection.utils import ops as utils_ops
from object_detection.utils import visualization_utils as vis_util

from geometry_msgs.msg import Quaternion

# Some Constants
CONNECTION_CHECK_PERIOD = 250 #ms
GUI_UPDATE_PERIOD = 20 #ms
DETECT_RADIUS = 4 # the radius of the circle drawn when a tag is detected

import time

class DroneVideoDisplay(QtGui.QMainWindow):
	StatusMessages = {
		DroneStatus.Emergency : 'Emergency',
		DroneStatus.Inited    : 'Initialized',
		DroneStatus.Landed    : 'Landed',
		DroneStatus.Flying    : 'Flying',
		DroneStatus.Hovering  : 'Hovering',
		DroneStatus.Test      : 'Test (?)',
		DroneStatus.TakingOff : 'Taking Off',
		DroneStatus.GotoHover : 'Going to Hover Mode',
		DroneStatus.Landing   : 'Landing',
		DroneStatus.Looping   : 'Looping (?)'
		}
	DisconnectedMessage = 'Disconnected'
	UnknownMessage = 'Unknown Status'
	
	def __init__(self):
		# Construct the parent class
		super(DroneVideoDisplay, self).__init__()

		# Setup our very basic GUI - a label which fills the whole window and holds our image
		self.setWindowTitle('AR.Drone Video Feed')
		self.imageBox = QtGui.QLabel(self)
		self.setCentralWidget(self.imageBox)

		# Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
		self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata) 
		
		# Subscribe to the drone's video feed, calling self.ReceiveImage when a new frame is received
		self.subVideo   = rospy.Subscriber('/ardrone/image_raw',Image,self.ReceiveImage)
		self.count_image = 0
		#self.subVideo   = rospy.Subscriber('/usb_cam/image_raw',Image,self.ReceiveImage)
		
		# Holds the image frame received from the drone and later processed by the GUI
		self.image = None
		self.imageLock = Lock()
		self.raw_image = None
		self.cv_image = None
		self.coordinates = None

		self.bridge = CvBridge()

		self.tags = []
		self.tagLock = Lock()
		
		# Holds the status message to be displayed on the next GUI update
		self.statusMessage = ''

		# Tracks whether we have received data since the last connection check
		# This works because data comes in at 50Hz but we're checking for a connection at 4Hz
		self.communicationSinceTimer = False
		self.connected = False

		# A timer to check whether we're still connected
		self.connectionTimer = QtCore.QTimer(self)
		self.connectionTimer.timeout.connect(self.ConnectionCallback)
		self.connectionTimer.start(CONNECTION_CHECK_PERIOD)
		
		# A timer to redraw the GUI
		self.redrawTimer = QtCore.QTimer(self)
		self.redrawTimer.timeout.connect(self.RedrawCallback)
		self.redrawTimer.start(GUI_UPDATE_PERIOD)
		
		self._num_classes = 90
		self._detection_graph = None
		self._sess = None
		self.category_index = None
		self._num_workers = -1
		
		# get an instance of RosPack with the default search paths
	        rospack = rospkg.RosPack()

	        self._tf_object_detection_path = \
	            rospack.get_path('drone') + \
	            '/src/object_detection'
	
	        self._path_to_ckpt = self._tf_object_detection_path + '/' + \
	            'ssd_mobilenet_v1_coco_2017_11_17' + '/frozen_inference_graph.pb'
	
	        # List of the strings that is used to add correct label for each box.
	        self._path_to_labels = self._tf_object_detection_path + '/' + \
	            'data/' + "mscoco_label_map.pbtxt"

		self.pubDetBox = rospy.Publisher('/ardrone/pubDetBox',Quaternion)
		self.DetBox = Quaternion()

		self.prepare()

	# Called every CONNECTION_CHECK_PERIOD ms, if we haven't received anything since the last callback, will assume we are having network troubles and display a message in the status bar
	def ConnectionCallback(self):
		self.connected = self.communicationSinceTimer
		self.communicationSinceTimer = False

	def RedrawCallback(self):
		if self.image is not None:
			# We have some issues with locking between the display thread and the ros messaging thread due to the size of the image, so we need to lock the resources
			self.imageLock.acquire()
			#print("redraw")
			try:			
					# Convert the ROS image into a QImage which we can display
					image = QtGui.QPixmap.fromImage(QtGui.QImage(self.image.data, self.image.width, self.image.height, QtGui.QImage.Format_RGB888))
					if len(self.tags) > 0:
						self.tagLock.acquire()
						try:
							painter = QtGui.QPainter()
							painter.begin(image)
							painter.setPen(QtGui.QColor(0,255,0))
							painter.setBrush(QtGui.QColor(0,255,0))
							for (x,y,d) in self.tags:
								r = QtCore.QRectF((x*image.width())/1000-DETECT_RADIUS,(y*image.height())/1000-DETECT_RADIUS,DETECT_RADIUS*2,DETECT_RADIUS*2)
								painter.drawEllipse(r)
								painter.drawText((x*image.width())/1000+DETECT_RADIUS,(y*image.height())/1000-DETECT_RADIUS,str(d/100)[0:4]+'m')
							painter.end()
						finally:
							self.tagLock.release()
			finally:
				self.imageLock.release()

			# We could  do more processing (eg OpenCV) here if we wanted to, but for now lets just display the window.
			self.resize(image.width(),image.height())
			self.imageBox.setPixmap(image)

		# Update the status bar to show the current drone status & battery level
		self.statusBar().showMessage(self.statusMessage if self.connected else self.DisconnectedMessage)

	def ReceiveImage(self,data):
		#print('Inside ReceiveImage')
		self.count_image = self.count_image + 1
		if self.count_image%6 == 0:
			
			# Indicate that new data has been received (thus we are connected)
			self.communicationSinceTimer = True

			# We have some issues with locking between the GUI update thread and the ROS messaging thread due to the size of the image, so we need to lock the resources
			self.imageLock.acquire()
			#print("We are Here")
			try:
				self.raw_image = data # Save the ros image for processing by the display thread
				self.cv_image = np.asarray(self.bridge.imgmsg_to_cv2(self.raw_image, desired_encoding="passthrough"))
			
				output_dict, category_index = self.detect(self.cv_image)
			
				#print("We are Here")
				start_detect = time.time()
				m_score = 0
				m_idx = -1
				for idx in range(len(output_dict['detection_classes'])):
					if (output_dict['detection_classes'][idx] == 1) and (output_dict['detection_scores'][idx] > 0.61):
						if output_dict['detection_scores'][idx] > m_score:
							m_score = output_dict['detection_scores'][idx]
							m_idx = idx
				if m_idx >= 0:
					print("Class Detected:", output_dict['detection_classes'][m_idx])
		                        self.coordinates = output_dict["detection_boxes"][m_idx]
					cv_box_image = self.visualize(self.cv_image, output_dict)
					self.image = self.bridge.cv2_to_imgmsg(cv_box_image, encoding="passthrough")
	 				#self.coordinates = self.coordinates[0]
					#print('Box: ', self.coordinates)
					self.DetBox.x = self.coordinates[0]
					self.DetBox.y = self.coordinates[1]
					self.DetBox.z = self.coordinates[2]
					self.DetBox.w = self.coordinates[3]
					self.pubDetBox.publish(self.DetBox)
				'''
				for class_i in output_dict['detection_classes']:
				  if class_i == 1 and output_dict['detection_scores'] > 50:
				    print("Class Detected:", class_i)
				    self.coordinates = output_dict["detection_boxes"]
				    cv_box_image = self.visualize(self.cv_image, output_dict)

				   
				    self.image = self.bridge.cv2_to_imgmsg(cv_box_image, encoding="passthrough")
 				    self.coordinates = self.coordinates[0]
				    print('Box: ', self.coordinates)
				    break'''
				end_detect = time.time()
				time_diff = end_detect - start_detect
				fout = open("/home/daksh/log_detect",'a')
				fout.write(str(time_diff)+"\n")
				fout.close()

	#TODO
	# Define a new variable self.raw_image
	# convert from ROS to CV cv_image = bridge.imgmsg_to_cv2(self.raw_image, desired_encoding="passthrough")
	# convert cv_image to numpy array
	# the ANN function goes here that takes in np_image variable, processes it and outputs box coordinates
	# Need to figure out: Combine box coordinates with cv_image = cv_image_box
	# convert cv_image _box back to ROS image self.image = cv2_to_imgmsg(cv_image_box, encoding="passthrough")
			finally:
				self.imageLock.release()
			if self.count_image >= 400:
				self.count_image = 0

	def visualize(self, image, output_dict):
		"""
		Draws the bounding boxes, labels and scores of each detection
		Args:
		image: (numpy array) input image
		output_dict (dictionary) output of object detection model
		Returns:
		image: (numpy array) image with drawings
		"""
		# Draw the bounding boxes
		vis_util.visualize_boxes_and_labels_on_image_array(
		    image,
		    output_dict["detection_boxes"],
		    output_dict["detection_classes"],
		    output_dict["detection_scores"],
		    self.category_index,
		    instance_masks=output_dict.get('detection_masks'),
		    use_normalized_coordinates=True,
		    line_thickness=5)

		return image

        def load_model(self):
		"""
		Loads the detection model
		Args:
		Returns:
		"""

		with self._detection_graph.as_default():
		    od_graph_def = tf.GraphDef()
		    with tf.gfile.GFile(self._path_to_ckpt , 'rb') as fid:
		        serialized_graph = fid.read()
		        od_graph_def.ParseFromString(serialized_graph)
		        tf.import_graph_def(od_graph_def, name='')

		label_map = label_map_util.load_labelmap(self._path_to_labels)
		categories = label_map_util.convert_label_map_to_categories(\
		    label_map, max_num_classes=self._num_classes, use_display_name=True)
		self.category_index = label_map_util.create_category_index(categories)

        def prepare(self):
		"""
		Prepares the model for detection
		Args:
		Returns:
		"""
		

		self._detection_graph = tf.Graph()

		self.load_model()

		# Set the number of workers of TensorFlow
		if self._num_workers == -1:
		    self._sess = tf.Session(graph=self._detection_graph)
		else:
		    session_conf = tf.ConfigProto(
		        intra_op_parallelism_threads=self._num_workers,
		        inter_op_parallelism_threads=self._num_workers,
		    )

		    self._sess = tf.Session(graph=self._detection_graph,
		        config=session_conf)


        def detect(self, image):
		"""
		Detects objects in the image given
		Args:
		image: (numpy array) input image
		Returns:
		output_dict (dictionary) Contains boxes, scores, masks etc.
		"""
		with self._detection_graph.as_default():
		 # Get handles to input and output tensors
		    ops = tf.get_default_graph().get_operations()
		    all_tensor_names = {output.name for op in ops for output in op.outputs}
		    tensor_dict = {}
		    for key in [
		       'num_detections', 'detection_boxes', 'detection_scores',
		       'detection_classes', 'detection_masks'
		       ]:
		        tensor_name = key + ':0'
		        if tensor_name in all_tensor_names:
		             tensor_dict[key] = tf.get_default_graph().get_tensor_by_name(
		             tensor_name)
		    if 'detection_masks' in tensor_dict:
		        # The following processing is only for single image
		        detection_boxes = tf.squeeze(tensor_dict['detection_boxes'], [0])
		        detection_masks = tf.squeeze(tensor_dict['detection_masks'], [0])
		        # Reframe is required to translate mask from box coordinates to image coordinates and fit the image size.
		        real_num_detection = tf.cast(tensor_dict['num_detections'][0], tf.int32)
		        detection_boxes = tf.slice(detection_boxes, [0, 0], [real_num_detection, -1])
		        detection_masks = tf.slice(detection_masks, [0, 0, 0], [real_num_detection, -1, -1])
		        detection_masks_reframed = utils_ops.reframe_box_masks_to_image_masks(
		            detection_masks, detection_boxes, image.shape[0], image.shape[1])
		        detection_masks_reframed = tf.cast(
		            tf.greater(detection_masks_reframed, 0.5), tf.uint8)
		        # Follow the convention by adding back the batch dimension
		        tensor_dict['detection_masks'] = tf.expand_dims(
		            detection_masks_reframed, 0)
		    image_tensor = tf.get_default_graph().get_tensor_by_name('image_tensor:0')

		    start = time.time()

		     # Run inference
		    output_dict = self._sess.run(tensor_dict,
		             feed_dict={image_tensor: np.expand_dims(image, 0)})

		    end = time.time()

		    #print end-start

		    # all outputs are float32 numpy arrays, so convert types as appropriate
		    output_dict['num_detections'] = int(output_dict['num_detections'][0])
		    output_dict['detection_classes'] = output_dict[
		        'detection_classes'][0].astype(np.uint8)
		    output_dict['detection_boxes'] = output_dict['detection_boxes'][0]
		    output_dict['detection_scores'] = output_dict['detection_scores'][0]
		    if 'detection_masks' in output_dict:
		        output_dict['detection_masks'] = output_dict['detection_masks'][0]
		return (output_dict, self.category_index)

	def ReceiveNavdata(self,navdata):
		# Indicate that new data has been received (thus we are connected)
		self.communicationSinceTimer = True

		# Update the message to be displayed
		msg = self.StatusMessages[navdata.state] if navdata.state in self.StatusMessages else self.UnknownMessage
		self.statusMessage = '{} (Battery: {}%)'.format(msg,int(navdata.batteryPercent))

		self.tagLock.acquire()
		try:
			if navdata.tags_count > 0:
				self.tags = [(navdata.tags_xc[i],navdata.tags_yc[i],navdata.tags_distance[i]) for i in range(0,navdata.tags_count)]
			else:
				self.tags = []
		finally:
			self.tagLock.release()

if __name__=='__main__':
	import sys
	rospy.init_node('ardrone_video_display')
	app = QtGui.QApplication(sys.argv)
	display = DroneVideoDisplay()
	display.show()
	status = app.exec_()
	rospy.signal_shutdown('Great Flying!')
	sys.exit(status)
