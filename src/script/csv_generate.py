#!/usr/bin/env python

import csv
from arduav.msg import Ardata, ArdataGPS

class DataRecord():
    def __init__(self):
        self.rowlabel = ["time", "baterry", "altitude" , "magX", "magY","magZ", "rotX", "rotY","rotZ","ax", "ay","az", "vx", "vy", "vz", "motor1", "motor2", "motor3", "motor4", "lat", "lng", "elevation","nbsat"] 
        self.ardata = Ardata()
        self.ardataGPS = ArdataGPS()
        self.data = []
        pass
    def captureData(self, t, ax, ay, az, vx, vy, vz, magX, magY, magZ, rotX, rotY, rotZ, motor1, motor2, motor3, motor4, lat, lng, elevation, nbsat):
        self.data[0] = self.
        pass
   def parserCSV(self):
        with open('fligthdata.csv', 'w') as file:
            writer = csv.writer(file)
            writer.writerow(rowlabel)
            for i in range(len(self.t)):
                writer.writerow([self.t[i], self.x[i], self.y[i],self.z[i]])
        pass



   
datarecord = DataRecord()
