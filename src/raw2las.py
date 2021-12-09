# -*- coding: utf-8 -*-
"""
Editor de Spyder

Este es un archivo temporal
"""

import sys
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import numpy as np
import os
import progressbar
# ROS
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply

class test:
    def __init__(self, filename):
        self.fileName=filename
        self.new_fileName=''
        #qx_r, qy_2, qz_r, qw_r = quaternion_from_euler(np.pi, 0, np.pi/2.0)
        #self.quad_r=np.matrix([[qx_r],[qy_r],[qz_r],[qw_r]])
        #self.rotate_z=self.Lidar2imu_yaw =np.matrix([[np.cos(np.pi),-np.sin(np.pi),0,0],[np.sin(np.pi),-np.cos(np.pi),0,0],[0,0,1,0],[0,0,0,1]])
        self.read()

    def quad2euler(self,qx,qy,qz,qw):
        orientation_list = [qx,qy,qz,qw]
        roll, pitch, yaw = euler_from_quaternion (orientation_list, axes='sxyz')
        return yaw, pitch, roll

    def matrixRotation(self,axis,angle):
        # -- Angle in rads
        if axis=='x':
            R=np.matrix([ [1,0,0,0], [0,np.cos(angle), -np.sin(angle), 0], [0, np.sin(angle), np.cos(angle), 0], [0,0,0,1] ])
        elif axis=='y':
            R=np.matrix([ [np.cos(angle),0,np.sin(angle),0], [0,1,0,0], [-np.sin(angle),0,np.cos(angle),0], [0,0,0,1] ])
        elif axis=='z':
            R=np.matrix([ [np.cos(angle),-np.sin(angle),0,0], [np.sin(angle), np.cos(angle),0,0], [0,0,1,0], [0,0,0,1] ])
        else:
            print("Axis no defined, use: x. y or z")
        return R

    def matrixTranslation(self,x_axis,y_axis,z_axis):
        #
        T=np.matrix([ [1,0,0,x_axis],[0,1,0,y_axis],[0,0,1,z_axis],[0,0,0,1] ])
        return T


    def newFile(self):
        self.new_fileName=self.fileName[:-4]+"_v2"+".txt"
        print("Nuevo Fichero: "+self.new_fileName+"\n")
        f = open(self.new_fileName,'w')
        f.close()


    def file_len(self, fname):
        i=0
    	with open(fname) as f:
    		for i, l in enumerate(f):
    			pass
    	return i + 1

    def read(self):
        f = self.fileName
        total = self.file_len(f)
        print("Total Lines: "+str(total))
        lines=0
        bar = progressbar.ProgressBar(maxval=total)
        with open(self.fileName) as infile:
            next(infile)
            next(infile)
            sample=0
            n=10
            X=[]
            Y=[]
            Z=[]
            A=[]
            ENU_aroundX=[]
            ENU_aroundY=[]
            ENU_aroundZ=[]
            correction = 1 # lineas 363-367 adquisition
            FLAG=0
            self.newFile()
            f = open(self.new_fileName,'a')
            bar.start()
            for line in infile:
                if sample%n == 0 and sample>20.0 and sample<30e6:
                    try:
                        dataLine=line.split('\t')
                        qx=float(dataLine[2])
                        qy=float(dataLine[3])
                        qz=float(dataLine[4])
                        qw=float(dataLine[5])
                        x=float(dataLine[6])
                        y=float(dataLine[7])
                        z=float(dataLine[8])
                        lat=float(dataLine[9])*np.pi/180.0
                        long=float(dataLine[10])*np.pi/180.0
                        alt=float(dataLine[11])
                        # ?
                        pc_x=float(dataLine[12])
                        pc_z=float(dataLine[13])
                        pc_y=float(dataLine[14])
                        pc_i=float(dataLine[15])*(np.power(pc_x,2)+np.power(pc_y,2)+np.power(pc_z,2))
                        if FLAG==0:
                            x_ini=x
                            y_ini=y
                            z_ini=z
                            FLAG=1
                        #if  pc_x>0 and np.sqrt(np.power(pc_x,2)+np.power(pc_y,2)+np.power(pc_z,2))<200:
                        radio=np.sqrt(np.power(pc_x,2)+np.power(pc_y,2)+np.power(pc_z,2))
                        if  radio<200 and radio>1 and pc_z>-10:
                            ECEF2ENU=np.matrix([ [-np.sin(long),np.cos(long),0,0], [-np.sin(lat)*np.cos(long), -np.sin(lat)*np.sin(long), np.cos(lat), 0],
                                [np.cos(lat)*np.cos(long), np.cos(lat)*np.sin(long), np.sin(lat), 0], [0,0,0,1] ])
                            ecef=np.matrix([[x-x_ini],[y-y_ini],[z-z_ini],[1]])
                            ENU=np.dot(ECEF2ENU,ecef)
                            X_ENU=ENU[0,0]
                            Y_ENU=ENU[1,0]
                            Z_ENU=ENU[2,0]
                            # Angles rotation
                            quat_ned=[qx,qy,qz,qw]
                            yaw_ned,pitch_ned,roll_ned = self.quad2euler(quat_ned[0],quat_ned[1],quat_ned[2],quat_ned[3])
                            quat_R = quaternion_from_euler(np.pi,0,-np.pi/2.0)
                            quat_enu = quaternion_multiply(quat_R, quat_ned)
                            yaw_enu,pitch_enu,roll_enu = self.quad2euler(quat_enu[0],quat_enu[1],quat_enu[2],quat_enu[3])
                            # local NED → ROS ENU: (x y z)→(y x -z) or (w x y z)→(y x -z w)  [https://github.com/mavlink/mavros/issues/216]
                            yaw_enu=-yaw_ned
                            pitch_enu=roll_ned
                            roll_enu=pitch_ned
                            # LiDAR transformations
                            old_pc= np.matrix([[pc_x],[pc_y],[pc_z],[1]])
                            new_pc= np.dot( self.matrixTranslation(0.0, 0.0, 8.0/100.0), old_pc)
                            new_pc= np.dot( self.matrixRotation("y", -np.pi*0.5), new_pc )
                            new_pc= np.dot( self.matrixRotation("x", np.pi), new_pc )
                            new_pc= np.dot( self.matrixRotation("x", roll_ned), new_pc )
                            new_pc= np.dot( self.matrixRotation("y", pitch_ned), new_pc )
                            new_pc= np.dot( self.matrixRotation("z", -yaw_ned-np.pi*0.5), new_pc )
                            new_pc= np.dot( self.matrixTranslation(X_ENU,Y_ENU,Z_ENU), new_pc)
                            #
                            # data accumulations
                            ENU_aroundY.append(pitch_enu*180.0/np.pi)
                            ENU_aroundX.append(roll_enu*180.0/np.pi)
                            ENU_aroundZ.append(yaw_ned*180.0/np.pi)
                            #dataLine_2=str(new_pc[0,0])+"\t"+str(new_pc[1,0])+"\t"+str(new_pc[2,0])+"\t"+str(pc_i)+"\n"
                            dataLine_2=str(lat)+"\t"+str(Y_ENU)+"\t"+str(long)+"\t"+str(alt)+"\n"
                            f.write(dataLine_2)
                    except:
                        print("An exception occurred")
                sample+=1
                lines+=n
                try:
                    bar.update(lines)
                except:
                    pass
        f.close()
        bar.finish()
        print("Samples: "+str(sample)+", "+str(len(X)))
        print("Converting to .LAS")
        input=self.new_fileName
        print("Output FILE:")
        output=self.new_fileName[:-4]+".las"
        print("\t"+str(output))
        instruction="wine /home/hmurcia/Downloads/LAStools/bin/txt2las.exe -i "+input+" -o "+output+" -parse xyzi -rescale 0.1 0.1 0.1"
        os.system(instruction)
        #instruction="rm "+input
        #os.system(instruction)

if __name__ == "__main__" :
    try:
        fileName=sys.argv[1]
    except:
        #fileName="/mnt/Turing/Harold/data/2019-11-14-16-23.txt"
        #fileName="/mnt/Turing/Harold/data/2019-11-14-16-48.txt"
        #fileName="/mnt/Turing/Harold/data/2019-11-14-16-58.txt"
        #fileName="/mnt/Turing/Harold/data/2019-11-14-17-04.txt"
        #fileName="/mnt/Turing/Harold/data/2019-11-22-15-19.txt"  #entrada garipa
        #fileName="/mnt/Turing/Harold/data/2019-11-22-15-11.txt"  # 69 to 78
        #fileName="/mnt/Turing/Harold/data/2019-11-26-01-28.txt"  #  puente exito
        #fileName="/mnt/Turing/Harold/data/2019-11-26-00-45.txt"   # Catedral
        #fileName="/mnt/Turing/Harold/data/2019-11-29-08-18.txt"     # Agrosavia 1
        #fileName="/mnt/Turing/Harold/data/2019-11-29-08-57.txt"     # Agrosavia 2
        fileName="/mnt/Harold/data/honda/2019-12-14-22-38.txt"  #Plaza
        fileName="/home/hmurcia/Desktop/data/segundo.txt" #samaria
        fileName="/home/hmurcia/Desktop/interp.txt" #samaria


    o1=test(fileName)
