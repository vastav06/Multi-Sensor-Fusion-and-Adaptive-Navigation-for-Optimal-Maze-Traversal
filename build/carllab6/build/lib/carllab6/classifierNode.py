#Authors: Natalie Davis and Vastav Bharambe
#ECE 7785 - Lab04
#Subscribes to the scan node. Detects the ranges and orientation of obstacles. Filter the LIDAR data to determine
#what measurements of the 360 deg are useful. Segment readings to be able to discern two obstacles apart.
#Publishes the vector pointing from the robot to the nearest point on the object.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import sys
import numpy as np
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg._point import Point
import csv
import time
from sklearn import svm
from std_msgs.msg import Int32
from geometry_msgs.msg._point import Point
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import LaserScan

class MinimalObjRangeDetector(Node): 

	def __init__(self):		
		#Creates the node.
		super().__init__('img_classifier') #this names the node

        #Declare that the minimal_video_subscriber node is subcribing to the /camera/image/compressed topic.
		self._video_subscriber = self.create_subscription(
				CompressedImage,
				'/camera/image/compressed',
				self._image_callback,
				1)

		self._reqClass = self.create_subscription(
				Int32,
				'reqClass',
				self.reqClass_callback,
				10)
		self._reqClass  # prevent unused variable warning
        # prevent unused variable warning

		self.reqClassVal = Int32()

        #Prevents unused variable warning.
		self._video_subscriber

		#Publisher for ret value
		self._classValPub = self.create_publisher(Int32, "classVal", 10)

        #Train image classifier
		imageDirectory = '/home/burger/carllab6/2022Simgs/'

		with open(imageDirectory + 'train.txt', 'r') as f:
			reader = csv.reader(f) 
			lines = list(reader)

		# this line reads in all images listed in the file in GRAYSCALE, and resizes them to 33x25 pixels
		train = np.array([np.array(self.edgeDetection(cv2.imread(imageDirectory +lines[i][0]+".jpg",1))) for i in range(len(lines))])

		# here we reshape each image into a long vector and ensure the data type is a float (which is what KNN wants)
		train_data = train.flatten().reshape(len(lines), 64*64*3)
		train_data = train_data.astype(np.float32)

		# read in training labels
		train_labels = np.array([np.int32(lines[i][1]) for i in range(len(lines))])


		### Train classifier
		self.clf = svm.SVC(kernel='linear')
		self.clf.fit(train_data, train_labels)

	def reqClass_callback(self, msg):
		self.reqClassVal = msg.data       
		print(f'Req Classifier Val = {self.reqClassVal}')

	def _image_callback(self, CompressedImage):	

		######################################################################
        #Test image against classifier                                       #
        ######################################################################
		self._imgBGR = CvBridge().compressed_imgmsg_to_cv2(CompressedImage, "bgr8")	
		test_img = self.edgeDetection(self._imgBGR)
		test_img = cv2.resize(test_img, (64,64))
		test_img = test_img.flatten().reshape(1, 64*64*3)
		test_img = test_img.astype(np.float32)

		#ret, results, neighbours, dist = knn.findNearest(test_img, k)
		dec = self.clf.decision_function(test_img)
		ret = np.argmax(dec , axis = 1)[0]

		#publish ret value
		classValMsg = Int32()
		classValMsg.data = int(ret)
		self._classValPub.publish(classValMsg)

		#Reset value to 0 so it does not classify image again
		self.reqClassVal = 0

		print(f'Image is classified as: {int(ret)}')


	def edgeDetection(self, img):
    	#Gray scale and blur the image for better edge detection
		originalImg = img
   
		gray = cv2.medianBlur(cv2.cvtColor(img, cv2.COLOR_RGB2GRAY), 5)

		gray = cv2.medianBlur(gray, 5)
		(thresh, blackAndWhiteImage) = cv2.threshold(gray, 73, 255, cv2.THRESH_BINARY)

		# Canny Edge Detection
		edges = cv2.Canny(blackAndWhiteImage, threshold1=150, threshold2=200) # Canny Edge Detection

		#Takes into account the non-zero edges from the canny output. If there are no edges, then return the original image
		pts = np.argwhere(edges>0)
		if(pts.size!=0):

			#Finds contours on the canny edges output image
			contours, hierarchy = cv2.findContours(image=edges, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)
			sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)
			#Used to display contours, if uncommented, will put a label on each contour and print out the area
			font = cv2.FONT_HERSHEY_COMPLEX
			ival=0
			bigContour = 0
			maxval = any
			for cnt in sorted_contours:
				#Used for x and y coordinates for placing font label on the image
				approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)

				#if the contour area is greater than a certain threshold, analyze it
				if cv2.contourArea(cnt) > 30.0:
					#cv2.drawContours(originalImg, [approx], 0, (0), 5)
					x = approx.ravel()[0]
					y = approx.ravel()[1]
					#cv2.putText(originalImg, f"CNT {ival}", (x, y), font, 1, (0))
					print(f'CNT area #{ival} {cv2.contourArea(cnt)}')

					#if the previous largest contour is smaller than the current, update it with the current largest
					#area value and update the maxval contour to be the current contour. this is used outside the for loop
					#for generating the bounding boxes
					if bigContour < cv2.contourArea(cnt):
						bigContour = cv2.contourArea(cnt)
						maxval = cnt
					ival+=1

			#Avoid failure of maxval not being set
			if ival == 0: maxval = cnt

			#makes a rectangle around the largest area contour. this is used to crop the final image
			(x,y,w,h) = cv2.boundingRect(maxval)
			cv2.rectangle(originalImg, (x,y), (x+w,y+h), (255, 0, 0), 2)
			cropped = img[y:y+h,x:x+w]

			# see the results
			#cv2.imshow('Cropped', cropped)
			#cv2.imshow('BoundingBox',originalImg)
			#cv2.waitKey(0)
			#cv2.imwrite('contours_none_image1.jpg', image_copy)
			#cv2.destroyAllWindows()

			return cv2.resize(cropped,(64,64),interpolation=cv2.INTER_LINEAR)
		else:
			return cv2.resize(img,(64,64),interpolation=cv2.INTER_LINEAR)

def main():
    #init routine needed for ROS2
	rclpy.init()
	img_classifier = MinimalObjRangeDetector()
	
    # Trigger callback processing
	rclpy.spin(img_classifier)

	#Clean up and shutdown.
	img_classifier.destroy_node()  
	rclpy.shutdown()


if __name__ == '__main__':
	main()