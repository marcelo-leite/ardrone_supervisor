#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from std_msgs.msg import String
import struct
import time
import serial


class ATcommand():
    
    def __init__(self):
        # self.move = Twist()
        pass
        
    def pcmd(self, move):
        # print(move)
        if move.linear.y == 0.0:
            move.linear.y = 0
        else:
            move.linear.y = -move.linear.y
        if move.linear.x == 0.0:
            move.linear.x = 0
        else:
            move.linear.x = -move.linear.x

        if move.linear.z == 0.0:
            move.linear.z = 0

        if move.angular.z == 0.0:
            move.angular.z = 0
        else:
            move.angular.z = -move.angular.z    

        speed_b = struct.pack('ffff', round(move.linear.y, 7), round(move.linear.x, 7), round(move.linear.z, 7), round(move.angular.z, 7))
        speed_i = struct.unpack('s', speed_b)
        cmd = "%#PCMD:" + "1," + str(speed_i[0]) + "," + str(speed_i[1]) + "," + str(speed_i[2]) + "," + str(speed_i[3]) + "~@$"
        # print(cmd)
        return cmd

class Ardrone():

    def __init__(self):


        # registering to publishers
        # self.pub_move= rospy.Publisher('/ardrone_data', String , queue_size=10)


        # registering to subscribers
        # rospy.Subscriber('/ardrone', String, self.callback_Navdata)
        rospy.Subscriber('joy', Joy, self.callback_joy)
        # rospy.Subscriber('/publisher', String, self.callback_pub)

    
       

        self.at =  ATcommand()
        #Variable Auxiliar
        self.rate = rospy.Rate(100) 	# 10Hz 
        self.stateAR = 0 		# State of fight
        self.move = Twist()		# Variable of movement
        self.speed = String

        # ARDUINO SERIAL
        self.arserial = serial.Serial('/dev/ttyACM0', 9600)
        time.sleep(1.8)

        #Variable Joy
        #TUSB CONTROL PS2 PC GENERIC "KNUP"
        self.data = Joy()
        self.data.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.data.buttons = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        #1  2   3   4   5   6   7   8   9       10      11  12 
        #1  2   3   4   L1  R1  L2  R2  SELECT  START   L3  R3 


    # FUCTIONS PUBLISHERS
    def takeoff(self):
        self.send_packet(str("%#REF:290718208~@$"))    
        self.rate.sleep()
    	

    def land(self):
        self.send_packet(str("%#REF:290717696~@$"))   
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
        else:
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
            # print(self.speed)
            self.send_packet(str(self.speed))
            self.rate.sleep()
    
       
	
    def send_packet(self, msg):
        self.arserial.write(str(msg))

        print(self.arserial.readline())
        self.rate.sleep()

    # FUCTIONS SUBSCRIBERS
        # def callback_Navdata(self, navmsg):
        #     print
    def callback_joy(self, msg_joy):
        self.data = msg_joy
        

    def callback_pub(self, msg):
        pass


if __name__ == '__main__':

	# initialize the node

    rospy.init_node('arduav', anonymous=False)

	# instantiate the class
    try:

        drone = Ardrone()
        print("Control Arduav Started")

        while not rospy.is_shutdown():
            drone.move_joy()
            

    except rospy.ROSInterruptException: 
	    pass
