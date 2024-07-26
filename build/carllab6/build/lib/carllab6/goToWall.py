#Authors: Natalie Davis and Vastav Bharambe
#ECE 7785 - Lab04
#Subscribes to local coordinate lidar vector being published from the getObjectRange.py file.
#Subcribes to the odom node, which determines the robot's global position from onboard sensors.
#Stores goal locations in an array. The first goal must be stopped at for 10 seconds within a 10 cm radius.
#The second goal must be stopped at for 10 seconds within a 15 cm radius. The third goal must be stopped at
#for 10 seconds within a 20 cm radius. The robot cannot hit any obstacles and reach the final destination within 
#2 minutes and 30 seconds.

from cmath import pi
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg._point import Point
from geometry_msgs.msg._twist import Twist
from std_msgs.msg._string import String
from nav_msgs.msg import Odometry
#Note: find TF2 import if used
from std_msgs.msg import Float32
from std_msgs.msg import Int32
import time

wayPoints = {'loc1' : [1.65, 0.0], 'loc2' : [1.65, 1.4], 'loc3' : [0.0, 1.4]}
kpang = 1/16
kplin = 1/2

class MinimalRotateRobo(Node):

    def __init__(self):
        super().__init__('rotateRobo')

        ######################################################################
        #Setting up the subscriber for the angular position and obj distance #
        ######################################################################
        self._localLidarVal = self.create_subscription(
            Float32,
            'vectorRange',
            self.localLidarVal_callback,
            10)
        self._localLidarVal  # prevent unused variable warning
        # prevent unused variable warning

        self._classifierVal = self.create_subscription(
            Int32,
            'classVal',
            self.classifierVal_callback,
            10)
        self._classifierVal  # prevent unused variable warning
        # prevent unused variable warning

        self._odomVal = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self._localLidarVal  # prevent unused variable warning
        # prevent unused variable warning

        ######################################################################
        #Setting up the publisher for the cmd_vel                            #
        ######################################################################
        self._velPublisher = self.create_publisher(Twist, '/cmd_vel', 1)

        #Publisher for ret value
        self._reqClassPub = self.create_publisher(Int32, "reqClass", 10)

        ######################################################################
        #Initializing update_Odometry values                                 #
        ######################################################################
        self.Init = True
        self.X = Float32()
        self.Y= Float32()
        self.Z= Float32()
        self.globalAng = Float32()
        Mrot = np.zeros((2,2))
        self.Init_ang = Float32()
        self.g_X = Float32()
        self.g_Y = Float32()
        self.x_way, self.y_way = wayPoints['loc1']

        self.x_starting = 0.0
        self.y_starting = 0.0

        ######################################################################
        #Initializing roboPos values                                         #
        ######################################################################
        self.goal_robotx = Float32()
        self.goal_roboty = Float32()
        self.goal_robotTheta = Float32()
        self.wayPointCount = 0
        self.distCur = 1
        self.robotAng = Twist()
        self.robotLin = Twist()
        self.angFlag = 0
        self.minLidarDist = Float32()
        self.gtgFlag = True
        self.wfAngBool = True
        self.wf90Bool = True
        self.wfComplete = True
        self.wfX = Float32()
        self.wfY = Float32()
        self.turnAngle = Float32()
        self.curClassNum = Int32()

    def classifierVal_callback(self, msg):
        self.classVal = msg.data       
        print(f'Classifier Val = {self.classVal}')

    def localLidarVal_callback(self, msg):
        self.minLidarDist = msg.data       
        print(f'LidarVal = {self.minLidarDist}')

    def odom_callback(self, msg):
        self.update_Odometry(msg)

        print(f'Odom Callback has x: {self.g_X}, y: {self.g_Y}, ang: {self.globalAng}, x_way: {self.x_way}, y_way: {self.y_way}')

        distance = Float32()
        distance = 0.5

        if self.minLidarDist < distance:
            #publish 0 vel and ang
            self.robotLin.linear.x = 0.0
            self.robotAng.angular.z = 0.0
            self._velPublisher.publish(self.robotLin)  
            self._velPublisher.publish(self.robotAng)  

            #grab the classification value and determine action
            if self.wfAngBool == True:

                #publish request for classifier
                # reqClassMsg = Int32()
                # reqClassMsg.data = 1
                # self._reqClassPub.publish(reqClassMsg)

                # print('Requested classifier value')

                # time.sleep(1)

                # print(f'Image classified as {self.classVal}')

                if self.classVal == 0: #empty wall
                    self.curClassNum = self.classVal
                    print('Empty wall')
                elif self.classVal == 1: #left
                    print('Turning left')
                    self.curClassNum = self.classVal
                    self.turnAngle = pi/2
                elif self.classVal == 2: #right
                    print('Turning right')
                    self.curClassNum = self.classVal
                    self.turnAngle = 3*pi/2
                elif self.classVal == 3: #do not enter
                    print('Turning 180')
                    self.curClassNum = self.classVal
                    self.turnAngle = pi
                elif self.classVal == 4: #stop
                    print('Turning 180')
                    self.curClassNum = self.classVal
                    self.turnAngle = pi
                elif self.classVal == 5: #goal
                    print('At the goal')
                    self.curClassNum = self.classVal
                    exit()
            
            if self.curClassNum == 1 or 2 or 3 or 4:
                self.turnBot(self.turnAngle)

        elif self.wfComplete: #Once we have completely turned the right direction, go back to going straight linearly
            self.goToWall()
            
            #Reset for next time the lidar distance is too small
            self.wfAngBool = True
            self.wf90Bool = True
        else:
            self.turnBot(self.turnAngle)


    def goToWall(self):
        print('In go to wall')
        ######################################################################
        #Angular position calculation                                        #
        ######################################################################
        thetaGoalRads = 0
        thetaCurRads = self.globalAng
        thetaSumRads = thetaGoalRads - thetaCurRads

        while thetaSumRads <= -pi:
            thetaSumRads = thetaSumRads + 2*pi
        while thetaSumRads >= pi:
            thetaSumRads = thetaSumRads - 2*pi
        
        thetaSum = np.rad2deg(thetaSumRads)

        self.robotAng.angular.z = kpang*(thetaSum)

        print(f'Angular Vel before trimming: {self.robotAng.angular.z}')

        if self.robotAng.angular.z > 2.84:
            self.robotAng.angular.z = 2.0
        elif self.robotAng.angular.z < -2.84:
            self.robotAng.angular.z = -2.0

        self._velPublisher.publish(self.robotAng)

        print(f'Angular Vel: {self.robotAng.angular.z}, ThetaCur is {np.rad2deg(thetaCurRads)}, thetaGoal - thetaCur = {thetaSum}, thetaGoal = {np.rad2deg(thetaGoalRads)}')

        if -2 < thetaSum < 2: #Update this to be smaller probably
            self.angFlag = 1
        else:
            self.angFlag = 0

        ######################################################################
        #Linear velocity commanding                                          #
        ######################################################################
        if self.angFlag == 1:
            #If within the right angle range, move forward at a constant speed
            self.robotLin.linear.x = 0.12
            print(f'In goToWall commanding angular: {self.robotAng.angular.z}, commanding linear: {self.robotLin.linear.x}')
            self._velPublisher.publish(self.robotLin)

    def turnBot(self, turnAngle):
        print('In wall following')
        if self.wfAngBool == True:
            self.robotLin.linear.x = 0.0
            self.robotAng.angular.z = 0.0
            self._velPublisher.publish(self.robotLin)  
            self._velPublisher.publish(self.robotAng)   
            self.wallFollowTheta = self.globalAng + turnAngle #turn 90 deg with pi/2
            print(f'Turning {turnAngle} deg, self wall follow theta is: {self.wallFollowTheta} and global ang is {self.globalAng}')

            self.wfAngBool = False
            self.wfComplete = False

        if self.wf90Bool:
            print('In self wf90')
            thetaSumRads = self.wallFollowTheta - self.globalAng

            while thetaSumRads <= -pi:
                thetaSumRads = thetaSumRads + 2*pi
            while thetaSumRads >= pi:
                thetaSumRads = thetaSumRads - 2*pi
            
            #rad2deg and p controller and vel clipping
            thetaSum = np.rad2deg(thetaSumRads)

            print(f'Thetacur: {np.rad2deg(self.globalAng)} and goal angle {np.rad2deg(self.wallFollowTheta)}, thetasum: {thetaSum}')

            self.robotAng.angular.z = kpang*(thetaSum)
                
            if self.robotAng.angular.z > 2.84:
                self.robotAng.angular.z = 2.0
            elif self.robotAng.angular.z < -2.84:
                self.robotAng.angular.z = -2.0

            #commanding angular velocity
            self._velPublisher.publish(self.robotAng)

            if -2 < (np.rad2deg(thetaSum)) < 2: #probably update this to be smaller
                self.wf90Bool = False
                self.wfComplete = True
                #After classifying, reset the odom
                self.Init = True

                time.sleep(2) 


    def update_Odometry(self, Odom):
        position = Odom.pose.pose.position
        
        #Orientation uses the quaternion aprametrization.
        #To get the angular position along the z-axis, the following equation is required.
        q = Odom.pose.pose.orientation
        orientation = np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

        if self.Init:
            #The initial data is stored to by subtracted to all the other values as we want to start at position (0,0) and orientation 0
            self.Init = False
            self.Init_ang = orientation
            self.globalAng = self.Init_ang
            Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        
            self.X = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y
            self.Y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y
            self.Z = position.z

        Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        

        #We subtract the initial values
        self.g_X = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y - self.X
        self.g_Y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y - self.Y
        self.globalAng = orientation - self.Init_ang

        print(f'Coordinates, x: {self.g_X}, y: {self.g_Y}, angle: {self.globalAng}, orientation: {orientation}, init angle: {self.Init_ang}')
        

def main(args=None):
    rclpy.init(args=args)

    rotateRobo = MinimalRotateRobo()

    rclpy.spin(rotateRobo)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rotateRobo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
