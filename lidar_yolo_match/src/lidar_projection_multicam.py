#!/usr/bin/env python2
import numpy as np
from numpy.linalg import inv, qr
import math 
import pandas as pd
import matplotlib.pyplot as plt
import datetime
import scipy
import time
import socket
import datetime
import cv2

from velodyne_capture_multicam import init_velo_socket, get_pointcloud, get_cam_pointcloud
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Circle
from utils_data import *
from utils_data import CalibFields

# TODO: Conversion matrix test 
class calibration:
    def __init__(self):
        self.h = None
        self.h_2 = None
        self.h_3 = None
        self.camera_matrix = None
        self.camera_matrix_2 = None
        self.camera_matrix_3 = None
    def tranform_cal(self):
        # lidar to camera 1
        self.camera_matrix = np.matrix([[769.534729,0.000000,653.262129],[0.000000,788.671204,360.453779],[0.000000,0.000000,1.000000]])
        final_rotation = np.matrix([[0.00525551,-0.999785,-0.0200588],[-0.00248811,0.020046,-0.999796],[0.999983,0.00530435,-0.00238222]])
        rotation_inv = inv(final_rotation)
        t=np.array([[0.0431043],[-0.0184322],[0.025641],[1]])
        zero = [0,0,0]
        final_rotation_test = np.vstack((final_rotation,zero))
        h_test = inv(np.hstack((final_rotation_test,t)))
        # print("h_test:",h_test)
        t=np.array([[h_test[0,3] ],[h_test[1,3]],[h_test[2,3]]]) 
        self.h=np.hstack((rotation_inv.T,t)) # stacked [R | t] 3*4 matrix
        
        # TODO: test lidar multi cam fusion select 
        # lidar to camera 2 right 
        self.camera_matrix_2  = np.matrix([[755.469543,0.000000,621.616852],[0.000000,763.467896,386.262318],[0.000000,0.000000,1.000000]])
        final_rotation_2 = np.matrix([[-0.858299,0.51315,0.000899662],[0.0780764,0.132324,-0.988127],[-0.507176,-0.848038,-0.153638]])
        rotation_inv_2 = inv(final_rotation_2)

        t_2=np.array([[0.00627183],[0.0100153],[0.0256176],[1]])
        zero = [0,0,0]
        final_rotation_test_2 = np.vstack((final_rotation_2,zero))
        h_test_2 = inv(np.hstack((final_rotation_test_2,t_2)))
        # print("h_test:",h_test)
        t_2=np.array([[h_test_2[0,3] ],[h_test_2[1,3]],[h_test_2[2,3]]]) 

        self.h_2=np.hstack((rotation_inv_2.T,t_2)) # stacked [R | t] 3*4 matrix
        # lidar to camera 3 left
        self.camera_matrix_3 = np.matrix([[1108.952148,0.000000,636.424646],[0.000000,1114.169434,416.902895],[0.000000,0.000000,1.000000]])
        final_rotation_3 = np.matrix([[0.866997,0.498306,-0.00278755],[0.0215986,-0.0431667,-0.998834],[-0.497846 ,0.865926,-0.0481881]])
        rotation_inv_3 = inv(final_rotation_3)
        
        t_3=np.array([[0.00521209],[-0.0790055],[0.0274718],[1]])
        zero = [0,0,0]
        final_rotation_test_3 = np.vstack((final_rotation_3,zero))
        h_test_3 = inv(np.hstack((final_rotation_test_3,t_3)))
        # print("h_test:",h_test)
        t_3=np.array([[h_test_3[0,3] ],[h_test_3[1,3]],[h_test_3[2,3]]])  

        self.h_3=np.hstack((rotation_inv_3.T,t_3)) # stacked [R | t] 3*4 matrix

    def lidar_cam_fusion(self,cam_num,pcl_point):
        if cam_num == 0:
            F = np.matmul((self.h),(pcl_point))
            cv_points = np.matmul((self.camera_matrix),(F))/F[2,:]

        elif cam_num == 1:
            F_2 = np.matmul((self.h_2),(pcl_point))
            cv_points = np.matmul((self.camera_matrix_2),(F_2))/F_2[2,:]
        elif cam_num == 2:
            F_3 = np.matmul((self.h_3),(pcl_point))
            cv_points = np.matmul((self.camera_matrix_3),(F_3))/F_3[2,:]
        return cv_points


if __name__ == '__main__':
    # Init sockets
    PORT = 2368
    soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    soc.bind(('', PORT))

    cal = calibration()
    cal.tranform_cal()
    cam = input("Enter the camera number to be calibrated 0~2:\n")

    if cam == 0:
        cap = cv2.VideoCapture(0)
    elif cam == 1:
        cap = cv2.VideoCapture(1)
    elif cam == 2:
        cap = cv2.VideoCapture(2)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FPS, 30)
    
    if not cap.isOpened():
        raise IOError("cannot open webcam")
    while 1:
        pcl = get_cam_pointcloud(soc,cam)
        # Get frame
        flag, frame = cap.read()
        cv2.imshow('frame', frame)	
        # Practical vlp16 manual xyz frame
        X= pcl[:,0] 
        Y= pcl[:,1]
        Z= pcl[:,2]
        distance = pcl[:,3]
        # make A matrix (x y z)
        size= len(X)

        X1= np.matrix.transpose(X)
        Y1= np.matrix.transpose(Y)
        Z1= np.matrix.transpose(Z)
        W= np.ones(size)
        W1= np.matrix.transpose(W)
        A=[X1,Y1,Z1]
        pcl_matrix= np.matrix([X1,Y1,Z1,W1])
    #----------------0818
        # Convert to vlp16 ros coordinate system output
        A=[X1,Y1,Z1,W1]
        real_vlp_to_ros = np.matrix([[0,-1,0,0],[1,0,0,0],[0,0,1,0],[0,0,0,1]])
        pcl_matrix = np.matmul((real_vlp_to_ros),(A))
        #-------------
        cv_points = cal.lidar_cam_fusion(cam,pcl_matrix)
        #cv2.circle(frame, (50,100), 10, (255,0,0), thickness=-1)
        for x in np.nditer(cv_points, flags = ['external_loop'], order = 'F'): 
            # print((x[0]))
            if int(x[0])<1280 and int(x[1])<720 :
                # 1280 -> right/u # 720 down/v  
                cv2.circle(frame, (int(x[0]),int(720-x[1])), 1, (255,0,0), thickness=-1)
                # print(x)
        cv2.imshow('frame', frame)

        c = cv2.waitKey(1)
        if c == 27:
            break

    cap.release()
    cap.destroyAllWindows()