#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from std_msgs.msg import String
import struct

class ATcommand():
    
    def __init__(self):
        # self.move = Twist()
        pass
        
    def pcmd(self, move):
        # print(move)
        speed_b = struct.pack('ffff', round(move.linear.x, 7), round(move.linear.y, 7), round(move.angular.z, 7), round(move.linear.z, 7))
        speed_i = struct.unpack('iiii', speed_b)
        cmd = "#:PCMD:" + str(speed_i[0]) + "," + str(speed_i[1]) + "," + str(speed_i[2]) + "," + str(speed_i[3]) + ":@"
        # print(cmd)
        return cmd

class Ardrone():

    def __init__(self):


        # registering to publishers
        self.pub_move= rospy.Publisher('/ardrone_data', String , queue_size=10)

        # self.comport = serial.Serial('/dev/ttyACM0', 9600)
        # registering to subscribers
        # rospy.Subscriber('/ardrone', String, self.callback_Navdata)
        rospy.Subscriber('joy', Joy, self.callback_joy)

    
       


        #Variable Auxiliar
        self.at =  ATcommand()
        self.rate = rospy.Rate(50) 	# 10Hz 
        self.stateAR = 0 		# State of fight
        self.move = Twist()		# Variable of movement
        self.speed = String
        #Variable Joy
        #TUSB CONTROL PS2 PC GENERIC "KNUP"
        self.data = Joy()
        self.data.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.data.buttons = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        #1  2   3   4   5   6   7   8   9       10      11  12 
        #1  2   3   4   L1  R1  L2  R2  SELECT  START   L3  R3 


    # FUCTIONS PUBLISHERS
    def takeoff(self):
        self.pub_move.publish(str("#*T*@"))
        self.rate.sleep()
    	

    def land(self):
        self.pub_move.publish(str("#*L*@"))
        self.rate.sleep()
       
    def move_joy(self):

	# Axes Functions -- "axes values to variables of movement"
        if self.data.buttons[4] == 1:
            self.takeoff()
            # pass
        elif self.data.buttons[6] == 1 and self.data.buttons[7] == 1:
            self.land()
        #     pass
       		# Velocity Linear
        elif self.data.buttons[10] == 1:
            self.move.linear.x = self.data.axes[1]/2.0
            self.move.linear.y = self.data.axes[0]/2.0
            # if self.data.buttons[7] == 1:
            self.move.linear.z = self.data.axes[3]/2.0
            # else:
            #     self.move.linear.z = 0.0
            
        
                # Velocity Angular
            self.move.angular.x = 0.0
            self.move.angular.y = 0.0
        
            self.move.angular.z = self.data.axes[2]/2.0

          

        # Publisher Velocity
            # self.speed = "#*C+[" + str(round(self.move.linear.x, 7)) + "," + str(round(self.move.linear.y, 7)) + "," + str(round(self.move.linear.z, 7)) + "," + str(round(self.move.angular.z, 7)) + "]*@"
            self.speed = self.at.pcmd(self.move)
            self.pub_move.publish(str(self.speed))
            self.rate.sleep()
    # FUCTIONS SUBSCRIBERS
    # def callback_Navdata(self, navmsg):
    #     print
        
    def callback_joy(self, msg_joy):
        self.data = msg_joy


if __name__ == '__main__':

	# initialize the node

    rospy.init_node('arduav_joy', anonymous=True)

	# instantiate the class
    try:

        drone = Ardrone()
        print("Control Joy Started")

        while not rospy.is_shutdown():
            drone.move_joy()

    except rospy.ROSInterruptException: 
	    pass
