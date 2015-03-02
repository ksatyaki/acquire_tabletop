#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy
import cv2
import time
import sys
import getopt

is_complete = False

chessboard_rows = 0
chessboard_cols = 0

# Use the mkgray function that converts our image to one that can be processed.
def makegrey(msg):
	br = CvBridge()
	# as cv_bridge automatically scales, we need to remove that behavior
	if msg.encoding.endswith('16'):
		mono16 = br.imgmsg_to_cv2(msg, "mono16")
		mono8 = mono16.astype(numpy.uint8)
		return mono8
	elif 'FC1' in msg.encoding:
	# floating point image handling
		
		img = br.imgmsg_to_cv2(msg, "passthrough")
		_, max_val, _, _ = cv2.minMaxLoc(img)
		if max_val > 0:
			scale = 255.0 / max_val
			mono_img = (img * scale).astype(np.uint8)
		else:
			mono_img = img.astype(np.uint8)
		return mono_img
	else:
		return br.imgmsg_to_cv2(msg, "mono8")


def initChessBoard(argv):

	print argv
	global chessboard_rows
	global chessboard_cols
	
	print "Chessboard!"
	try:
		opts, args = getopt.getopt(argv, "hr:c:", ["help", "rows=", "cols="])
		print opts, args
	except getopt.GetoptError:
		sys.exit(2)
	
	for opt, arg in opts:
		if opt in ("-r", "--rows"):
			global chessboard_rows
			chessboard_rows = int(arg)
		if opt in ("-c", "--cols"):
			global chessboard_cols
			chessboard_cols = int(arg)
	
	print 'Chessboard with ', chessboard_rows, ' rows and ', chessboard_cols, ' columns'
			
			
def computeCorners(gray):
	retval, corners = cv2.findChessboardCorners(gray, (chessboard_rows,chessboard_cols))
	print "*$*$*$*************"
	print corners

	if retval == True:
		global is_complete
		is_complete = True
		rgb = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
		cv2.drawChessboardCorners(rgb, (11,7), corners, retval)
		cv2.imshow('grayscale',rgb)
	else:
		cv2.imshow('scale', gray)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		return

# cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding="passthrough")
def imageCallback(data):
	computeCorners(makegrey(data))

if __name__ == '__main__':
	rospy.init_node ('calibrator')
	initChessBoard(rospy.myargv(argv=sys.argv[1:]))
	image_sub = rospy.Subscriber("image", Image, imageCallback)
	while(is_complete == False and not rospy.is_shutdown()):
		print 'Waiting'
		time.sleep(1)
		
