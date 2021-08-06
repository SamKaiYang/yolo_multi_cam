#!/usr/bin/env python2
import sys
import rospy
import os
import numpy as np
import math 
import pandas as pd
import matplotlib.pyplot as plt
import datetime
import scipy
import time
import socket
import datetime

# from client import init_yolo_socket, get_bounding_boxes
from velodyne_capture_v3 import init_velo_socket, get_pointcloud, get_cam_pointcloud
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Circle
from collisionNew import Human, collision_detection
###
from yolo_detection.msg import ROI_array
from yolo_detection.msg import ROI
'''
code to get lidar point cloud, get bounding boxes for that frame, 
and predict a collision using Kalman filter (in progress)
	
'''
from darknet_ros_msgs.msg import BoundingBox
from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import ObjectCount

from yolo_detection.msg import ROI_array
from yolo_detection.msg import ROI
import time
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray


from sensor_msgs.msg import Image 
from yolo_detection.msg import cam_output
# Initialization info
human_dim = {
        "length": 0.5,
        "width": 2.1
        }

fake_obj = {
        "length": 4.5,
        "width": 2.1,
        "x": 22.0,
        "y": 15.0,
        "angle": 75,
        "speed": 6.0
        }

# Set up sockets:
HOST = "192.168.1.201"
PORT = 2368
# TCP_IP = '127.0.0.1'
# TCP_PORT = 8080
# s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# s.connect((TCP_IP, TCP_PORT))
soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
soc.bind(('', PORT))

# For matrix values
xr = 95 * math.pi/180
yr = 10 * math.pi/180  
zr = 0 * math.pi/180

# Get R matrix
Xr = np.matrix([[1,0,0],[0,math.cos(xr),-1*math.sin(xr)],[0,math.sin(xr),math.cos(xr)]])
Yr = np.matrix([[math.cos(yr),0,math.sin(yr)],[0,1,0],[-1*math.sin(yr),0,math.cos(yr)]])
Zr = np.matrix([[math.cos(zr),-1*math.sin(zr),0],[math.sin(zr),math.cos(zr),0],[0,0,1]])

F = np.matrix([[935,0,0],[0,935,0],[225,375,1]])

R = np.matmul(Zr,Yr)
R= np.matmul(R,Xr)

# Transpose matrix?
T = np.matrix([[0.9],[0],[-1.32]])
T1=np.matrix.transpose(T)

# Initialize previous x,y,t
prev_x = 0
prev_y = 0
prev_t = 0

# Initialize human objects: 
# human location and dimensions (CHANGE)
my_human = Human(0,0,human_dim["width"],human_dim["length"],0)
# Detected object (CHANGE)
other_human = Human(0,0,fake_obj["width"],fake_obj["length"],0)

# Get x, y, z, and distance data from point cloud

# xcenter = 0
# ycenter = 0
# numObjects = 0

#############
image1 = None
image2 = None
image3 = None
cnt = 0
data_count = 0

cam_out_num = None
# YOLO V4
i = 0
obj_num = 0
test_boundingboxes = None
_test_boundingboxes = None
cam_out_num = None
_cam_out_num = None
def Yolo_callback(data):
	global obj_num, cam_out_num, test_boundingboxes

	test_boundingboxes = data.bounding_boxes
	cam_out_num = data.cam_out
	obj_num = len((data.bounding_boxes))
            
def YoloCount_callback(data):
    global data_count
    data_count = data.count

def Image1_callback(data):
    global cnt, image1, image2, data_count, cam_out_num, test_boundingboxes , _cam_out_num, _test_boundingboxes
    image1 = data 
    if cnt == 0:
        pub_image.publish(image1)

        cam_num = cam_output()
        cam_num = 0
        pub_cam_num.publish(cam_num)

        if cam_out_num == 0:
			_cam_out_num = cam_out_num 
			_test_boundingboxes = test_boundingboxes 
			cnt = 1

def Image2_callback(data):
    global cnt, image1, image2, data_count, cam_out_num, test_boundingboxes , _cam_out_num, _test_boundingboxes
    image2 = data 
    if cnt == 1:
        pub_image.publish(image2)

        cam_num = cam_output()
        cam_num = 1
        pub_cam_num.publish(cam_num)

        if cam_out_num == 1:
			_cam_out_num = cam_out_num 
			_test_boundingboxes = test_boundingboxes
			cnt = 0
if __name__ == '__main__':
    #global boxes
	argv = rospy.myargv()
	rospy.init_node('lidar_yolo_match', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes,Yolo_callback)
	rospy.Subscriber("/darknet_ros/found_object",ObjectCount,YoloCount_callback)
	rospy.Subscriber("/camera1/usb_cam1/image_raw",Image,Image1_callback)
	rospy.Subscriber("/camera2/usb_cam2/image_raw",Image,Image2_callback)
    # rospy.Subscriber("/camera3/usb_cam3/image_raw",Image,Image3_callback)
	pub_cam_num  =  rospy.Publisher("/cam_num", cam_output, queue_size=10)
	pub_image = rospy.Publisher("/usb_cam/image_raw", Image, queue_size=10)

	while not rospy.is_shutdown():
		
		# _test_boundingboxes = test_boundingboxes
		# _cam_out_num = cam_out_num 
		if _test_boundingboxes == None:
			pass
		else:
			# start = datetime.datetime.now()
			print("cam_out_num",_cam_out_num)
			pcl = get_cam_pointcloud(soc,_cam_out_num)
			# elapsed = (datetime.datetime.now() - start).microseconds
			# # print('pcl time: ', elapsed)
			X= pcl[:,0]
			Y= pcl[:,1]
			Z= pcl[:,2]
			distance = pcl[:,3]

			# make A matrix (x y z)
			size= len(X)
			X1= np.matrix.transpose(X)
			Y1= np.matrix.transpose(Y)
			Z1= np.matrix.transpose(Z)
			A= np.matrix([X1, Y1 ,Z1])
			print(A.shape)
			T2= np.repeat(T1,size,axis=0)
			T2= np.matrix.transpose(T2)

			# Multiply matrices (lidar points in pixel coordinates)
			c2 = np.matmul((F), (R))
			c2 = .25*np.matmul((c2),(A+T2))
			#print("matrix calculations: ", (datetime.datetime.now()-start).microseconds)
			#print('objects', numObjects)
			#print(datetime.datetime.now())
			
			for i in range(len(_test_boundingboxes)):
				print("len(_test_boundingboxes):",len(_test_boundingboxes))
				if _test_boundingboxes[i].Class == "person":
					xmin = _test_boundingboxes[i].xmin
					ymin = _test_boundingboxes[i].ymin
					xmax = _test_boundingboxes[i].xmax
					ymax = _test_boundingboxes[i].ymax

					# # Center of box
					xcenter = (xmin+xmax)/2.0
					ycenter = (ymin+ymax)/2.0

					# c3 = c2
					# Bounding box
					start = datetime.datetime.now()

					B = np.square((c2[0,:]-xcenter))+ np.square((c2[1,:]-ycenter))

					# Get lidar points in bounding box
					#points = []

					#points = [[X[i], Y[i], distance[i]] for i in range(c2_T.shape[0]) if (c2_T[i,0] > left and c2_T[i,0] < right and c2_T[i,1] > top and c2_T[i,1] < bottom)]
					# for i in range(c2_T.shape[0]):
					# 	if c2_T[i,0] > left and c2_T[i,0] < right and c2_T[i,1] > top and c2_T[i,1] < bottom:
					# 		points.append([X[i], Y[i], distance[i]])
					# elapsed = (datetime.datetime.now() - start).microseconds
					# print(elapsed/1000)
					# print(len(points)) 

					# Get index of lidar point for detected object
					index0 = int(np.argmin(B, axis=1))
					
					# print(index0)
					#print('y', Y[index0])
					#print("Index of center point is:", index0)

					# printing x,y, and distance for detected objects
					print('x:{:.2f} y:{:.2f} distance: {:.2f}'.format(X[index0], Y[index0], distance[index0]));

					
					# Get inputs ready for prediction: [x,y,vx,vy,dt]
					# x = X[index0]
					# y = Y[index0]
					# t = time.time()

					# # Account for first instance:
					# if (prev_t == 0):
					# 	dt = 1
					# 	vx = 0
					# 	vy = 0
					# # Else, update:
					# else:	
					# 	dt = t - prev_t
					# 	vx = (x - prev_x)/(t_in - prev_t)
					# 	vy = (y_- prev_x)/(t)
					# 	prev_x = x
					# 	prev_y = y
					# 	prev_t = t

					# # Code from collisionNew.py:
					# other_human.update_locarray([x, y, vx, vy, dt])
					# #print('distance: {:.2f}'.format(distance[index0]))  

					# my_human.update_speed()
					# other_human.update_object()

					# if(collision_detection(my_human,other_human)):
					# 	print('ALERT!!!')
					# 	# alert()
											
		print(' ')
		rate.sleep()
	rospy.spin()
