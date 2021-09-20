#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from std_msgs.msg import String
# from ardrone_autonomy.msg import Navdata
from arduav.msg import Ardata, ArdataGPS

import struct
import time
import serial
import csv
   
class CaptureNavdata():
    def __init__(self):
        # self.ARserial = ARserial
        self.ARserial = serial.Serial('/dev/ttyUSB0', 57600)
        self.rate = rospy.Rate(10) 
        self.navdata = Ardata()
        self.gps = ArdataGPS()
        self.rowlabel = [   "time", 
                            "battery", 
                            "altitude" , 
                            "pressure",
                            "magX", 
                            "magY",
                            "magZ", 
                            "rotX", 
                            "rotY",
                            "rotZ",
                            "ax", 
                            "ay",
                            "az", 
                            "vx", 
                            "vy", 
                            "vz", 
                            "motor1", 
                            "motor2", 
                            "motor3", 
                            "motor4", 
                            "lat", 
                            "lng", 
                            "elevation",
                            "nbsat"
                        ] 
        self.datacol = dict()
        for key in self.rowlabel:
            self.datacol[key] = []
        self.coletaState = 0

        # Publisher
        self.pub_navdata = rospy.Publisher('/ardrone/ardata', Ardata, queue_size=10)
        self.pub_gps = rospy.Publisher('/ardrone/ardata_gps', ArdataGPS, queue_size=10)
        
    def parserCSV(self):
        with open('flightdata.csv', 'w') as file:
            
            writer = csv.writer(file)
            writer.writerow(self.rowlabel)
            for i in range(len(self.datacol["time"])):
                arraytemp = []
                for key in self.rowlabel:
                    arraytemp.append(self.datacol[key][i])
                writer.writerow(arraytemp)

        pass
    def coletaData(self):
        self.datacol["time"].append(rospy.get_time())
        self.datacol["battery"].append(self.navdata.batteryPercent)
        self.datacol["altitude"].append(self.navdata.altd)
        self.datacol["pressure"].append(self.navdata.pressure)
        self.datacol["magX"].append(self.navdata.magX)
        self.datacol["magY"].append(self.navdata.magY)
        self.datacol["magZ"].append(self.navdata.magZ)
        self.datacol["rotX"].append(self.navdata.rotX)
        self.datacol["rotY"].append(self.navdata.rotY)
        self.datacol["rotZ"].append(self.navdata.rotZ)
        self.datacol["ax"].append(self.navdata.ax)
        self.datacol["ay"].append(self.navdata.ay)
        self.datacol["az"].append(self.navdata.az)
        self.datacol["vx"].append(self.navdata.vx)
        self.datacol["vy"].append(self.navdata.vy)
        self.datacol["vz"].append(self.navdata.vz)
        self.datacol["motor1"].append(self.navdata.motor1)
        self.datacol["motor2"].append(self.navdata.motor2)
        self.datacol["motor3"].append(self.navdata.motor3)
        self.datacol["motor4"].append(self.navdata.motor4)
        self.datacol["lat"].append(self.gps.latitude)
        self.datacol["lng"].append(self.gps.longitude)
        self.datacol["elevation"].append(self.gps.elevation)
        self.datacol["nbsat"].append(self.gps.nbsat)
        

    def setData(self, data):
        self.sequence = data[0]

        self.ardroneState(data[1])       
        self.baterry = data[2]
        self.theta = data[3]       
        self.phi = data[4]          
        self.psi = data[5]            

        self.altitude = data[6]           
        self.pression = data[7]

        self.v = [data[8], data[9], data[10]]    
  

        self.phys_accs = [data[11], data[12], data[13]]
        self.magneto_raw = [data[14], data[15], data[16]]

        self.wind_speed = data[17]		
        self.wind_angle = data[18]	


        self.motor = [data[19], data[20], data[21], data[22]]
    
        self.link_quality = data[23]
    
        self.latitude = data[24]
        self.longitude = data[25]
        self.elevation = data[26]
        self.gps_state = data[27]
        self.nbsat = data[28]
        self.video_buffer = []
       

        s = "ccccBBHIHHHHIIBBBBIIHBBBBBBIBBBBBBBBBBBB"
        video_byte = ""
        
        # s = list(s)
        # for i in range(29, len(data)):
        #     self.video_buffer.append(data[i])
            
         
        # for i in range(0, len(self.video_buffer)):

        #     video_byte = video_byte + struct.pack(s[i], self.video_buffer[i])
            
        # print(struct.calcsize("ccccBBHIHHHHIIBBBBIIHBBBBBBIBBBBBBBBBBBB"))
        # print(len(s))
        # for i in s:
           
        # print(len( self.video_buffer))


    def ardroneState(self, state):
        # https://github.com/venthur/python-ardrone/blob/master/libardrone.py

        self.drone_state = dict()
        self.drone_state['fly_mask']             = state       & 1 # FLY MASK : (0) ardrone is landed, (1) ardrone is flying
        self.drone_state['video_mask']           = state >>  1 & 1 # VIDEO MASK : (0) video disable, (1) video enable
        self.drone_state['vision_mask']          = state >>  2 & 1 # VISION MASK : (0) vision disable, (1) vision enable */
        self.drone_state['control_mask']         = state >>  3 & 1 # CONTROL ALGO (0) euler angles control, (1) angular speed control */
        self.drone_state['altitude_mask']        = state >>  4 & 1 # ALTITUDE CONTROL ALGO : (0) altitude control inactive (1) altitude control active */
        self.drone_state['user_feedback_start']  = state >>  5 & 1 # USER feedback : Start button state */
        self.drone_state['command_mask']         = state >>  6 & 1 # Control command ACK : (0) None, (1) one received */
        self.drone_state['fw_file_mask']         = state >>  7 & 1 # Firmware file is good (1) */
        self.drone_state['fw_ver_mask']          = state >>  8 & 1 # Firmware update is newer (1) */
        self.drone_state['fw_upd_mask']          = state >>  9 & 1 # Firmware update is ongoing (1) */
        self.drone_state['navdata_demo_mask']    = state >> 10 & 1 # Navdata demo : (0) All navdata, (1) only navdata demo */
        self.drone_state['navdata_bootstrap']    = state >> 11 & 1 # Navdata bootstrap : (0) options sent in all or demo mode, (1) no navdata options sent */
        self.drone_state['motors_mask']          = state >> 12 & 1 # Motor status : (0) Ok, (1) Motors problem */
        self.drone_state['com_lost_mask']        = state >> 13 & 1 # Communication lost : (1) com problem, (0) Com is ok */
        self.drone_state['vbat_low']             = state >> 15 & 1 # VBat low : (1) too low, (0) Ok */
        self.drone_state['user_el']              = state >> 16 & 1 # User Emergency Landing : (1) User EL is ON, (0) User EL is OFF*/
        self.drone_state['timer_elapsed']        = state >> 17 & 1 # Timer elapsed : (1) elapsed, (0) not elapsed */
        self.drone_state['angles_out_of_range']  = state >> 19 & 1 # Angles : (0) Ok, (1) out of range */
        self.drone_state['ultrasound_mask']      = state >> 21 & 1 # Ultrasonic sensor : (0) Ok, (1) deaf */
        self.drone_state['cutout_mask']          = state >> 22 & 1 # Cutout system detection : (0) Not detected, (1) detected */
        self.drone_state['pic_version_mask']     = state >> 23 & 1 # PIC Version number OK : (0) a bad version number, (1) version number is OK */
        self.drone_state['atcodec_thread_on']    = state >> 24 & 1 # ATCodec thread ON : (0) thread OFF (1) thread ON */
        self.drone_state['navdata_thread_on']    = state >> 25 & 1 # Navdata thread ON : (0) thread OFF (1) thread ON */
        self.drone_state['video_thread_on']      = state >> 26 & 1 # Video thread ON : (0) thread OFF (1) thread ON */
        self.drone_state['acq_thread_on']        = state >> 27 & 1 # Acquisition thread ON : (0) thread OFF (1) thread ON */
        self.drone_state['ctrl_watchdog_mask']   = state >> 28 & 1 # CTRL watchdog : (1) delay in control execution (> 5ms), (0) control is well scheduled */
        self.drone_state['adc_watchdog_mask']    = state >> 29 & 1 # ADC Watchdog : (1) delay in uart2 dsr (> 5ms), (0) uart2 is good */
        self.drone_state['com_watchdog_mask']    = state >> 30 & 1 # Communication Watchdog : (1) com problem, (0) Com is ok */
        self.drone_state['emergency_mask']       = state >> 31 & 1 # Emergency landing : (0) no emergency, (1) emergency */

    def readSerialWithVideo(self):
        # print("Iniciando")
        s = []
        permission = False
        while True:
            assinatura = ''
            if(self.ARserial.read(1) == 'N'):
                assinatura = assinatura + 'N'

                for i in range(6):
                    assinatura = assinatura + str(self.ARserial.read(1))

                assinatura = str(assinatura)
                if(assinatura == 'NAVDATA'):
                    permission = True
                    break
                else:
                    permission = False
            
        if(permission):
            # print(len(data))
            # signature = "7c"
            fligth_data = "=3I3f2i3f3f3f2f4BI3d2I"
            video_data = "4c2BHI4H2I4B2IH4B2BI6B6B"
            todo = fligth_data + video_data
            data = self.ARserial.read(struct.calcsize(todo))
            
            data_u = struct.unpack(todo, data)
            for x in range(28, len(data_u)):
                s.append(str(data_u[x]))

            self.setData(data_u)
            print("")
            print("Sequencia %s"%(self.sequence))
            print("Bateria %s"%(self.baterry))
            print("Pressao %s"%(self.pression))
            print("Altitude %s"%(self.altitude))
            print("Angulo %s"%(self.psi/1000))

            
            # print("Angulo %s"%(self.psi))
            # print(self.video_buffer)
            # print(self.phi/1000)
            # print(self.latitude)
            # print(self.longitude)
            # print(self.drone_state['fly_mask'])
            # print(self.drone_state['navdata_demo_mask'])
            # print(self.drone_state['emergency_mask'])
        
    def readSerial(self):
        # print("Iniciando")
        s = []
        permission = False
        while True:
            assinatura = ''
            if(self.ARserial.read(1) == 'N'):
                assinatura = assinatura + 'N'

                for i in range(6):
                    assinatura = assinatura + str(self.ARserial.read(1))

                assinatura = str(assinatura)
                if(assinatura == 'NAVDATA'):
                    permission = True
                    break
                else:
                    permission = False
            
        if(permission):

            fligth_data = "=3I3f2i3f3f3f2f4BI3d2I"
            data = self.ARserial.read(struct.calcsize(fligth_data))
            data_u = struct.unpack(fligth_data, data)
            for x in range(28, len(data_u)):
                s.append(str(data_u[x]))

            self.setData(data_u)            
            self.pubNavdata()
            
    def printData(self):
        print("Sequencia %s"%(self.sequence))
        print("Bateria %s"%(self.baterry))
        print("Pressao %s"%(self.pression))
        print("Altitude %s"%(self.altitude))
        print("Angulo %s"%(self.psi/1000))
        print("Latitue %s"%(self.latitude))
        print("Longitude %s"%(self.longitude))
        print("Motor1 %s"%(self.motor[0]))
        print("Motor2 %s"%(self.motor[1]))
        print("Motor3 %s"%(self.motor[2]))

        print("mx %s"%(self.magneto_raw[0]))
        print("mx %s"%(self.magneto_raw[1]))
        print("mx %s"%(self.magneto_raw[2]))
        
        # print("Angulo %s"%(self.psi))
        # print(self.video_buffer)
        # print(self.phi/1000)
        # print(self.latitude)
        # print(self.longitude)
        # print(self.drone_state['fly_mask'])
        # print(self.drone_state['navdata_demo_mask'])
        # print(self.drone_state['emergency_mask'])  
        #     
    def pubNavdata(self):
        self.navdata.header.seq = self.sequence
        self.navdata.altd = self.altitude
        self.navdata.batteryPercent = self.baterry
        self.navdata.pressure = self.pression;

        self.navdata.ax = self.phys_accs[0];
        self.navdata.ay = self.phys_accs[1];
        self.navdata.az = self.phys_accs[2];
        
        self.navdata.magX = self.magneto_raw[0]
        self.navdata.magY = self.magneto_raw[1]
        self.navdata.magZ = self.magneto_raw[2]

        self.navdata.vx = self.v[0];
        self.navdata.vy = self.v[1];
        self.navdata.vz = self.v[2];

        self.navdata.rotX = self.phi;
        self.navdata.rotY = self.theta;
        self.navdata.rotZ = self.psi;

        self.navdata.motor1 = self.motor[0]
        self.navdata.motor2 = self.motor[1]
        self.navdata.motor3 = self.motor[2]
        self.navdata.motor4 = self.motor[3]

        self.gps.latitude = self.latitude
        self.gps.longitude = self.longitude
        self.gps.elevation = self.elevation
        self.gps.nbsat = self.nbsat
        if(self.coletaState == 1):
            self.coletaData()
            print("Coletando")
        elif(self.coletaState == 2):
            print("Gerando CSV")
            self.parserCSV()
            self.coletaState = 0
        else:
            pass
        self.pub_navdata.publish(self.navdata)
        self.pub_gps.publish(self.gps)
        # self.printData()
        self.rate.sleep()
        
class Control():
    def __init__(self):
        
        #INSTANCE CLASS
        # self.ARserial = ARserial
        self.ARserial = serial.Serial('/dev/ttyUSB0', 57600)
        self.captureNavdata = CaptureNavdata()
        self.rate = rospy.Rate(10) 	# 10Hz 
        
        

        #ROS SUBSCRIBER
        rospy.Subscriber('joy', Joy, self.callbackJoy)
        self.v = Twist()		# Variable of movement

        #TUSB CONTROL PS2 PC GENERIC "KNUP"
        self.data = Joy()
        self.data.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.data.buttons = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def takeoff(self):
        cmd_p = struct.pack('=3cB4c', 'C','M','D',4,'T','A','K','E')
        # Send cmd
        self.ARserial.write(cmd_p)
        self.rate.sleep()
        pass
        

    def land(self):
        cmd_p = struct.pack('=3cB4c', 'C','M','D',4,'L','A','N','D')
        # Send cmd
        self.ARserial.write(cmd_p)
        self.rate.sleep()
        pass
    def move(self):
        # Velocity Linear
        self.v.linear.x = (-1)*self.data.axes[1]/2.0
        self.v.linear.y = (-1)*self.data.axes[0]/2.0
        self.v.linear.z = self.data.axes[2]/2.0
        
    
        # Velocity Angular
        self.v.angular.x = 0.0
        self.v.angular.y = 0.0
        self.v.angular.z = self.data.axes[3]/2.0
        print(self.v.linear.x)
        print(self.v.linear.y)
        print(self.v.linear.z)
        print(self.v.angular.z)
        
        # Pack cmd
        cmd_p = struct.pack('=3cB4c4f', 'C','M','D',20,'P','C','M','D',self.v.linear.x,self.v.linear.y,self.v.linear.z,self.v.angular.z)
        # Send cmd
        self.ARserial.write(cmd_p)
        self.rate.sleep()
        pass
       
    def gamerControl(self):
        # Axes Functions -- "axes values to variables of movement"
        if self.data.buttons[4] == 1:
            # Aircraft Takeoff
            self.takeoff()
            print("Decolando")

        elif self.data.buttons[6] == 1 and self.data.buttons[7] == 1 :
            # Aircraft Land
            self.land()
            print("Pousando")
        elif self.data.buttons[0] == 1:
            self.captureNavdata.coletaState = 1
        elif self.data.buttons[1] == 1:
            self.captureNavdata.coletaState = 2
        elif self.data.buttons[2] == 1:
            self.captureNavdata.coletaState = 0

        # elif self.captureNavdata.drone_state['fly_mask'] == 1:
        # # else:
        #     # Aircraft Move
        #     print("Movendo-se")
        #     self.move()

    def callbackJoy(self, msg_joy):
        self.data = msg_joy
         




class Ardrone():

    def __init__(self):
        ARserial = serial.Serial('/dev/ttyUSB0', 57600)
        self.control = Control()
        


if __name__ == '__main__':
    
	# initialize the node

    rospy.init_node('arduav', anonymous=False)

	# instantiate the class
    try:

        drone = Ardrone()
        print("Control Arduav Started")

        while not rospy.is_shutdown():
            drone.control.captureNavdata.readSerial()
            drone.control.gamerControl()

    except rospy.ROSInterruptException: 
	    pass
