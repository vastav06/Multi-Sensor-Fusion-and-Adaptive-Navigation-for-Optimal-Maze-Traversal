#Authors: Natalie Davis and Vastav Bharambe
#ECE 7785 - Lab04
#Subscribes to the scan node. Detects the ranges and orientation of obstacles. Filter the LIDAR data to determine
#what measurements of the 360 deg are useful. Segment readings to be able to discern two obstacles apart.
#Publishes the vector pointing from the robot to the nearest point on the object.

from gc import set_debug
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import math
import sys

import numpy as np
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg._point import Point
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg._float32 import Float32
from std_msgs.msg._string import String

class MinimalObjRangeDetector(Node): 

	def __init__(self):		
		#Creates the node.
		super().__init__('obj_range') #this names the node

        ######################################################################
        #Setting up the subscriber for the LIDAR                             #
        ######################################################################
        
    	#Declare that the obj_range node is subscribing to the \scan topic.
		qos_profile = QoSProfile(depth=1)
		qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
		qos_profile.durability = QoSDurabilityPolicy.VOLATILE
		self._lidarSub = self.create_subscription(LaserScan, '/scan',self.lidar_callback,qos_profile) 
		self._lidarSub # prevent unused variable warning

        ######################################################################
        #Setting up the publisher -publishing angluar position               #
        #and distance fo the object                                          #
        ######################################################################
		self._vecRangePub = self.create_publisher(Float32, "vectorRange", 10)
		self._vecStringPub = self.create_publisher(String, "vectorString", 10)

	def lidar_callback(self, msg):
		vecRangeMsg = Float32()
		vecStringMsg = String()

		#Ranges
		#Front left: 30 - 59
		#Front right: 177 - 206
		#Front: 206 - 236, 0 - 30
		front =[]
		frontleft=[]
		frontRight=[]
		A=[]
		FL=[]
		FR=[]
		Front=[]
		for i in range(len(msg.ranges)):
			A.append(msg.ranges[i])

		Front= np.concatenate((A[0:20],A[-20:]))
		Front = [i for i in (Front) if np.isnan(i)==False]
		Z=np.sort(Front)
		Front= Z[:20]
		minDist = np.average(Front)
		print(Front)

		angleString = 'front'

		vecRangeMsg.data = minDist
		vecStringMsg.data = angleString

		print(f'C = {minDist}, angleString = {angleString}')
		
		self._vecRangePub.publish(vecRangeMsg)
		self._vecStringPub.publish(vecStringMsg)





def main():
    #init routine needed for ROS2
	rclpy.init()
	obj_range = MinimalObjRangeDetector()
	
    # Trigger callback processing
	rclpy.spin(obj_range)

	#Clean up and shutdown.
	obj_range.destroy_node()  
	rclpy.shutdown()


if __name__ == '__main__':
	main()