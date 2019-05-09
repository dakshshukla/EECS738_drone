# EECS738_drone

This is the group project for EECS 738 course, Spring 2019 at KU.

The team members are as follows:

Daksh Shukla, Ben Liu, Patrick McNamee, Tim Fox, Caio Vigo

The final presentation is in the "ppt" folder.

_______________________________________________________________________________________________

This is a ROS package named "drone" and depends on tensorflow package
with python 2.7. ROS must be installed on the linux machine before running this software.

The "drone" package depends on ardrone_autonomy ROS package which should also be installed
in the ROS directory or can be in the same catkin workspace as the "drone" package.

Tested with ROS Kinetic kame and Ubuntu 16.04.

To run the project:

1. Connect Parrat AR drone through WiFi to your computer (preferably Ubuntu 16.04 machine)
2. Run the nodes:
	roslaunch drone drone.launch

A QT GUI should show up in about 5 seconds and should start displaying the images at 5Hz from the front 720p camera. The display may stop working if a human is not detected in front of the drone.
The terminal will start printing out the box location in terms of percentage from the left side of the image axis. It will also display roll commands.

The basic algorithm wors as follows:

1. Subscribe to image
2. Pass every "nth" (6th) image
3. frequency = 30/n (= 5 Hz)
4. Convert ROS image to CV image
5. Detect classes = ANN(image)
6. If class == 1 && det_score > 30%
7. find max score index
8. Use index
	- get box = [y_min, x_min,y_max, x
	_max]
	- visualize(image + box)
	- Make ROS message
	- Publish ROS topic
9. Subscribe to box data
10. Compute error w.r.t center
11. Compute & Publish roll commads
