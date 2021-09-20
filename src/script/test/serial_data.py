#!/usr/bin/env python

# from numpy.lib.function_base import append
import serial
import struct 
import time
from os import system, name
import cv2
# from dataclasses import dataclass
# import numpy as np
# from dataclasses import dataclass


# class (object):
    



class ARDrone():

    def __init__(self):
        self.arserial = serial.Serial('/dev/ttyUSB0', 57600)
        time.sleep(1.8)
        self.Vstream = ""
        pass
    def navData(self, data):
        self.sequence = data[0]


        # c_p = struct.pack('I', data[1])
        
        # self.ctrl_state = struct.unpack('4c', c_p)    
            
        self.ardroneState(data[1])       
        self.baterry = data[2]
        self.theta = data[3]       
        self.phi = data[4]          
        self.psi = data[5]            

        self.altitude = data[6]           
        self.pression = data[7]

        self.v = [data[8], data[9], data[10]]    
  

        self.phys_accs = [data[11], data[12], data[13]]
        self.phys_accs = [data[14], data[15], data[16]]

        self.wind_speed = data[17]		
        self.wind_angle = data[18]	


        self.motor = [data[19], data[20], data[21], data[22]]
    
        self.link_quality = data[23]
    
        self.latitude = data[24]
        self.longitude = data[25]
        self.elevation = data[26]
        self.gps_state = data[27]
        self.nbsat = data[28]
        
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
        
    def sendSerial(self):
        cmd = []
        k  = True
        while True:
            cmd = ['C','M','D',4,'T','A','K','E']
            
            if(k == 0):
                cmd_p = struct.pack('=3cB4c', 'C','M','D',4,'T','A','K','E')
                k += 1;
            elif(k == 1):
                cmd_p = struct.pack('=3cB4c', 'C','M','D',4,'L','A','N','D')
                
                k += 1;
            else:
                cmd_p = struct.pack('=3cB4c4i', 'C','M','D',20,'P','C','M','D',50,50,20,20)
                k = 0
            # self.arserial
            print(cmd_p)
            self.arserial.write(cmd_p)
            time.sleep(0.2)
    def readSerial(self):
        
        s = []
        permission = False
        while True:
            assinatura = ''
            if(self.arserial.read(1) == 'N'):
                assinatura = assinatura + 'N'
                for i in range(6):
                    assinatura = assinatura + str(self.arserial.read(1))
                    # print(self.arserial.read())
                    # time.sleep(1)
                    # print(i)

                assinatura = str(assinatura)
                if(assinatura == 'NAVDATA'):
                    # print("pronto")
                    permission = True
                    break
                else:
                    # print(assinatura)
                    permission = False
                # time.sleep(1)
        

        # data = assinatura + self.arserial.read(176)
        if(permission):
            data = self.arserial.read(180)
            # print(len(data))
            # signature = "7c"
            fligth_data = "=3I3f2i3f3f3f2f4BI3d2I"
            video_data = "4c2BHI4H2I4B2IH4B2BI6B6B"
            todo = fligth_data + video_data
            data_u = struct.unpack(todo, data)
            for x in range(28, len(data_u)):
                s.append(str(data_u[x]))
            v = ""
            y = 0
            # ldata = list(data)
            # print(data_u)
            self.navData(data_u)
            # print(self.arself.drone_state)

            print(self.baterry)
            print(self.latitude)
            print(self.drone_state['fly_mask'])
            print(self.drone_state['navdata_demo_mask'])
            print(self.drone_state['emergency_mask'])
       
            
      
    def unpackNavdata(self):
        pass

    
    def videoShow(self, video):
        cam = cv2.VideoCapture(video)
        running = True
        # while running:
        # get current frame of video
        running, frame = cam.read()
        # grayFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if running:
            # cv2.imshow('frame', frame)
            # cv2.imshow('video gray', grayFrame)
            cv2.imshow('video original', frame)
            if cv2.waitKey(1) & 0xFF == 27: 
                # escape key pressed
                running = False
        else:
            # error reading frame
            print 'error reading video feed'
        cam.release()
        cv2.destroyAllWindows()



if __name__ == '__main__':
        
    drone = ARDrone()
    while True:
        drone.readSerial()
        # navdata.sendSerial()