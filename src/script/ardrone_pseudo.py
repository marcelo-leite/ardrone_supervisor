#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Joy
import csv
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from std_msgs.msg import String
from arduav.msg import Ardata, ArdataGPS
from random import seed
from random import random
import serial

class Ardrone():
    def __init__(self):
        self.rate = rospy.Rate(3) 
        self.navdata = Ardata()
        self.gps = ArdataGPS()

        # Publisher
        self.pub_navdata = rospy.Publisher('/ardrone/ardata', Ardata, queue_size=10)
        self.pub_gps = rospy.Publisher('/ardrone/ardata_gps', ArdataGPS, queue_size=10)
        

            
    def pubNavdata(self):
        self.read_csv()
    
    def read_csv(self):
        with open('flightdata_final.csv') as file:
            spamreader = csv.reader(file, delimiter=',', quotechar='|')

            for row in spamreader:
                if(row[0] == "time"):
                    pass
                else:
                    # print(row[1])
                    self.navdata.batteryPercent = float(row[1])
                    self.navdata.altd = int(row[2])
                    self.navdata.pressure = int(row[3])
                    self.navdata.magX = float(row[4])
                    self.navdata.magY = float(row[5])
                    self.navdata.magZ = float(row[6])
                    self.navdata.rotX = float(row[7])
                    self.navdata.rotY = float(row[8])
                    self.navdata.rotZ = float(row[9])
                    self.navdata.ax = float(row[10])
                    self.navdata.ay = float(row[11])
                    self.navdata.az = float(row[12])
                    self.navdata.vx = float(row[13])
                    self.navdata.vy = float(row[14])
                    self.navdata.vz = float(row[15])
                    self.navdata.motor1 = int(row[16])
                    self.navdata.motor2 = int(row[17])
                    self.navdata.motor3 = int(row[18])
                    self.navdata.motor4 = int(row[19])
                    # self.gps.latitude = row[20]
                    # self.gps.longitude = row[21]
                    # self.gps.nbsat = row[22]
                    self.gps.latitude = -5.449284
                    self.gps.longitude = -47.402785
                    self.gps.elevation = 157.05
                    self.gps.nbsat = 9
                    self.pub_navdata.publish(self.navdata)
                    self.pub_gps.publish(self.gps)
                    self.rate.sleep()
    
    def test(self):

        self.navdata.ax = random()
        self.navdata.ay = random()
        self.navdata.az = random()
        battery = 10*random()

        self.navdata.batteryPercent = int(battery)*10
        # print(self.navdata.ax)
        self.pub_navdata.publish(self.navdata)
        
        self.rate.sleep()
                


if __name__ == '__main__':
    
	# initialize the node

    rospy.init_node('arduav', anonymous=False)

	# instantiate the class
    try:

        drone = Ardrone()
        print("Control Arduav Started")
        seed(1)
        while not rospy.is_shutdown():
            drone.pubNavdata()
            # drone.control.gamerControl()

    except rospy.ROSInterruptException: 
	    pass
