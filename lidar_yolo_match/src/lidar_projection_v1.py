#!/usr/bin/env python2
import numpy as np
import math 
import pandas as pd
import matplotlib.pyplot as plt
import datetime
import scipy
import time
import socket
import datetime
import cv2

from velodyne_capture_v3 import init_velo_socket, get_pointcloud, get_cam_pointcloud
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Circle
from utils_data import *
from utils_data import CalibFields

# Init sockets
PORT = 2368
soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
soc.bind(('', PORT))

cap = cv2.VideoCapture(0)
"""
# 0813 lidar to cam  Average transformation is:
0.99898    0.033474  -0.0302925 -0.00030335
-0.0362764    0.994598  -0.0972613   0.0610432
0.0268731    0.098261    0.994798   0.0157042
0           0           0           1

camera matrix
727.079910 0.000000 317.463020
0.000000 734.695266 240.018079
0.000000 0.000000 1.000000
camera projection
727.703247 0.000000 317.400100 0.000000
0.000000 737.639832 240.387843 0.000000
0.000000 0.000000 1.000000 0.000000
"""
# TODO: Conversion matrix test 
camera_matrix = np.matrix([[727.703247,0.000000,317.400100],[0.000000,737.639832,240.387843],[0.000000,0.000000,1.000000]])
transformation = np.matrix([[0.99898,0.033474,-0.0302925,-0.00030335],[-0.0362764,0.994598,-0.0972613,0.0610432],[0.0268731,0.098261,0.994798,0.0157042]])
# transformation = np.matrix([[0.99898,0.033474,-0.0302925],[-0.0362764,0.994598,-0.0972613],[0.0268731,0.098261,0.994798]])
# T = np.matrix([[-0.00030335],[0.0610432],[0.0157042]])
# test 0816  --------------
# camera_matrix = camera_matrix.T
# transformation = transformation
# -------
print("camera_matrix",camera_matrix)
print("transformation:",transformation)



""" 
<Coordinate system (Lidar)>
x: forward 
y: left
z: up

<Coordinate system (Camera/image)>
x: down
y: right
"""
# def loadCalib(f_int,f_ext):
#     dict_calib = {}
#     # Intrinsic matrix (3x4)
#     dict_calib[CalibFields.intrinsic] = f_int
#     # Extrinsic Matrix (4x4)
#     dict_calib[CalibFields.extrinsic] = f_ext
#     return dict_calib


# dict_calib = loadCalib(camera_matrix,transformation) 
# # im_height 480 , im_width 640

# #   points2D, pointsDist = project_lidar_to_img(self.cparam.dict_calib,
# #                                                     self.points,
# #                                                     self.im_height,
# #                                                     self.im_width)
# #         # Clip distance with min/max values
# #         for i_pt,pdist in enumerate(pointsDist):
# #             pointsDist[i_pt] = self.d_max if pdist>self.d_max \
# #                                else pdist if pdist>self.d_min \
# #                                else self.d_min
# #         # Rescale to 1~255 with 1/d value
# #         pointsDist = np.round(minmax_scale(1.0/pointsDist,
# #                                            1.0/self.d_max,1.0/self.d_min,
# #                                            1,255)).astype('uint8')
# #         # Color points w.r.t inversed distance values
# #         _CMAP = plt.get_cmap(cmap)
# #         pointsColor = np.array([_CMAP(1.0-pdist/255.0)[:3] \
# #                                 for pdist in pointsDist])
# #         plt.clf()
# #         plt.imshow(self.im)
# #         plt.scatter(points2D[:,1],points2D[:,0],
# #                     c=pointsColor,
# #                     s=1)
# #         plt.xlim(0,self.im_width-1)
# #         plt.ylim(self.im_height-1,0)
# #         plt.draw()

if not cap.isOpened():
	raise IOError("cannot open webcam")

while 1:
	# Get pointcloud
	# pcl = get_pointcloud(soc)
	# test cam num lidar match 
    # print(transformation[:,1])
    # print(transformation[0,1])
    pcl = get_pointcloud(soc)
	# Get frame
    flag, frame = cap.read()
    cv2.imshow('frame', frame)	
	
    X= pcl[:,0]
    Y= pcl[:,1]
    Z= pcl[:,2]
    print(X)
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
    # pcl_matrix= np.matrix([X1,Y1,Z1])
    # T1=np.matrix.transpose(T)
    # T2= np.repeat(T1,size,axis=0)
    # T2= np.matrix.transpose(T2)

    # # intrinsic * extrinsic
    F = np.matmul((camera_matrix),(transformation))
    cv_points = np.matmul((F),(pcl_matrix))

    # # test 1 point
    # pcl_points = np.matrix([X1,Y1,Z1,W1])
    # t_points = np.dot(pcl_points,transformation.T)
    # trans_mat = np.dot(camera_matrix,trans_mat)
    # points2D = np.dot(pcl_points,trans_mat.T)


    # pcl_points = pcl_points.T
    # points2D, pointsDist = project_lidar_to_img(dict_calib,pcl_points,480,640)
    # pcl_one = [20,30,20,1]
    # F_test = np.matmul((camera_matrix),(transformation))
    # cv_test = np.matmul((F_test),(pcl_one))
    # print("cv_test",cv_test)
    

    # Plot points on frame
    for x in np.nditer(cv_points, flags = ['external_loop'], order = 'F'): 
        # print((x[0]))
        cv2.circle(frame, (int(x[0]),int(x[1])), 1, (255,0,0), thickness=-1)
        #print(x)
    cv2.imshow('frame', frame)

    c = cv2.waitKey(1)
    if c == 27:
        break

cap.release()
cap.destroyAllWindows()