# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import numpy as np
from tqdm import tqdm
# ROS
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply

def matrixRotation(axis,angle):
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

def matrixTranslation(x_axis,y_axis,z_axis):
        T=np.matrix([ [1,0,0,x_axis],[0,1,0,y_axis],[0,0,1,z_axis],[0,0,0,1] ])
        return T

def newFile(new_fileName):
    print("Nuevo Fichero: "+new_fileName+"\n")
    f = open(new_fileName,'w')
    f.close()

def quad2euler(qx,qy,qz,qw):
        orientation_list = [qx,qy,qz,qw]
        roll, pitch, yaw = euler_from_quaternion (orientation_list, axes='sxyz')
        return yaw, pitch, roll

data=pd.read_csv("/home/hmurcia/Desktop/test.txt",sep='\t',header=0, skiprows=2)
data.rename(columns={ data.columns[0]: "scan_id" }, inplace = True)
data.rename(columns={ data.columns[1]: "time" }, inplace = True)
data.rename(columns={ data.columns[2]: "X" }, inplace = True)
data.rename(columns={ data.columns[3]: "Y" }, inplace = True)
data.rename(columns={ data.columns[4]: "Z" }, inplace = True)
data.rename(columns={ data.columns[5]: "qx" }, inplace = True)
data.rename(columns={ data.columns[6]: "qy" }, inplace = True)
data.rename(columns={ data.columns[7]: "qz" }, inplace = True)
data.rename(columns={ data.columns[8]: "qw" }, inplace = True)
data.rename(columns={ data.columns[9]: "lat" }, inplace = True)
data.rename(columns={ data.columns[10]: "longitude" }, inplace = True)
data.rename(columns={ data.columns[11]: "alt" }, inplace = True)
data.rename(columns={ data.columns[12]: "PC_X" }, inplace = True)
data.rename(columns={ data.columns[13]: "PC_Y" }, inplace = True)
data.rename(columns={ data.columns[14]: "PC_Z" }, inplace = True)
data.rename(columns={ data.columns[15]: "PC_intensity" }, inplace = True)
data.rename(columns={ data.columns[16]: "PC_ring" }, inplace = True)
data.rename(columns={ data.columns[17]: "PC_echo_layer" }, inplace = True)
print(data)
print("columns: ", data.columns)


L=len(data.X)
E=np.zeros([1,L])
N=np.zeros([1,L])
U=np.zeros([1,L])
ecef=np.ones([4,1])
PITCH=np.zeros([1,L])
ROLL=np.zeros([1,L])
YAW=np.zeros([1,L])
X_ini = data.X[0]
Y_ini = data.Y[0]
Z_ini = data.Z[0]
#
fileName = "point_cloud.txt"
newFile(fileName)
f = open(fileName,'a')
#
for k in tqdm(range(0,L,100)):
    longitude = data.longitude[k]*np.pi/180.0
    latitude  = data.lat[k]*np.pi/180.0
    ECEF2ENU=np.array([ [-np.sin(longitude),np.cos(longitude),0,0],
                         [-np.sin(latitude)*np.cos(longitude), -np.sin(latitude)*np.sin(longitude), np.cos(latitude), 0],
                         [np.cos(latitude)*np.cos(longitude), np.cos(latitude)*np.sin(longitude), np.sin(latitude), 0],
                         [0,0,0,1] ])
    ecef[0,0]=data.X[k]-X_ini
    ecef[1,0]=data.Y[k]-Y_ini
    ecef[2,0]=data.Z[k]-Z_ini
    ENU=np.dot(ECEF2ENU,ecef)
    X_ENU=ENU[0,0]
    Y_ENU=ENU[1,0]
    Z_ENU=ENU[2,0]
    E[0,k]=ENU[0,0]
    N[0,k]=ENU[1,0]
    U[0,k]=ENU[2,0]
    # Angles rotation
    quat_ned=[data.qx[k], data.qy[k], data.qz[k], data.qw[k]]
    yaw_ned,pitch_ned,roll_ned = quad2euler(quat_ned[0],quat_ned[1],quat_ned[2],quat_ned[3])
    quat_R = quaternion_from_euler(np.pi,0,-np.pi/2.0)
    quat_enu = quaternion_multiply(quat_R, quat_ned)
    yaw_enu,pitch_enu,roll_enu = quad2euler(quat_enu[0],quat_enu[1],quat_enu[2],quat_enu[3])
    # local NED ??? ROS ENU: (x y z)???(y x -z) or (w x y z)???(y x -z w)  [https://github.com/mavlink/mavros/issues/216]
    YAW[0,k]=-yaw_ned #yaw_enu
    ROLL[0,k]=roll_ned #pitch_enu
    PITCH[0,k]=pitch_ned #roll_enu
    # LiDAR transformations: from LiDAR to IMU
    old_pc= np.matrix([[data.PC_X[k]],[data.PC_Y[k]],[data.PC_Z[k]],[1]])
    new_pc= np.dot( matrixTranslation(0.0, 0.0, 8.0/100.0), old_pc)
    new_pc= np.dot( matrixRotation("y", -np.pi*0.5), new_pc )
    #new_pc= np.dot( matrixRotation("x", np.pi), new_pc )
    new_pc= np.dot( matrixRotation("x", roll_ned), new_pc )
    new_pc= np.dot( matrixRotation("y", pitch_ned), new_pc )
    new_pc= np.dot( matrixRotation("z", -yaw_ned-np.pi*0.5), new_pc )
    new_pc= np.dot( matrixTranslation(X_ENU,Y_ENU,Z_ENU), new_pc)
    dataLine=str(new_pc[0,0])+"\t"+str(new_pc[1,0])+"\t"+str(new_pc[2,0])+"\t"+str(data.PC_intensity[k])+"\n"
    f.write(dataLine)

f.close()
fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot3D(E[0,:], N[0,:], U[0,:],'.')
#ax.plot3D(data.X, data.Y, data.Z, 'red')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z');
fig.savefig('path.png')
