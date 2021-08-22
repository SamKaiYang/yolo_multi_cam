#!/usr/bin/env python2
import sys
import rospy
import os
import numpy as np
from numpy.linalg import inv, qr
import math 
import pandas as pd
import matplotlib.pyplot as plt
import datetime
import scipy
import time
import socket
'''
code to get lidar point cloud, get bounding boxes for that frame, 
and predict a collision using Kalman filter (in progress)
'''
from velodyne_capture_multicam import init_velo_socket, get_pointcloud, get_cam_pointcloud
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Circle
from collisionNew import Human, collision_detection

from darknet_ros_msgs.msg import BoundingBox
from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import ObjectCount

from sensor_msgs.msg import Image 
# Sub-execution work function
import threading

from lidar_yolo_match.msg import depth_alert
# from lidar_yolo_match.srv import alert_output, alert_outputResponse

class cal_class:
	def __init__(self, alert_calss,alert_calss_1,alert_calss_2):
		self.obj_num = 0
		self.boundingboxes = None
		self.cam_out_num = 0
		self.image_frame_id = None
		self.image1 = None
		self.image2 = None
		self.image3 = None
		self.image_cnt = 0
		self.soc = None
		self.T1 = None
		self.F = None
		self.R = None
		self.h = None
		self.camera_matrix = None
		self.h_2 = None
		self.camera_matrix_2 = None
		self.h_3 = None
		self.camera_matrix_3 = None
		self.cam_change_flag = False
		self.data_count = 0
		self.cam_num = None
		self.bounding = None
		self.bounding_num = None
		self.person_flag = False
		self.alert_calss = alert_calss
		self.alert_calss_1 = alert_calss_1
		self.alert_calss_2 = alert_calss_2
		# self.sub = rospy.Subscriber("chatter",String,self.callback)
		self.sub_bouding = rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes,self.Yolo_callback)
		self.sub_YOLOCount = rospy.Subscriber("/darknet_ros/found_object",ObjectCount,self.YoloCount_callback)
		self.sub_Image1 = rospy.Subscriber("/camera1/usb_cam1/image_raw",Image,self.Image1_callback)
		self.sub_Image2 = rospy.Subscriber("/camera2/usb_cam2/image_raw",Image,self.Image2_callback)
		self.sub_Image3 = rospy.Subscriber("/camera3/usb_cam3/image_raw",Image,self.Image3_callback)
		self.pub_image = rospy.Publisher("/usb_cam/image_raw", Image, queue_size=None)
		
	def vlp16_socket(self):
		# Set up sockets:
		HOST = "192.168.1.201"
		PORT = 2368
		self.soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.soc.bind(('', PORT))

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
	# lidar to camera 2 right 
		self.camera_matrix_2  = np.matrix([[755.469543,0.000000,621.616852],[0.000000,763.467896,386.262318],[0.000000,0.000000,1.000000]])
		final_rotation_2 = np.matrix([[-0.858299,0.51315,0.000899662],[0.0780764,0.132324,-0.988127],[-0.507176,-0.848038,-0.153638]])
		rotation_inv_2 = inv(final_rotation_2)
		t_2=np.array([[0.00627183],[0.0100153],[0.0256176],[1]])
		zero = [0,0,0]
		final_rotation_test_2 = np.vstack((final_rotation_2,zero))
		h_test_2 = inv(np.hstack((final_rotation_test_2,t_2)))
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
		t_3=np.array([[h_test_3[0,3] ],[h_test_3[1,3]],[h_test_3[2,3]]])  
		self.h_3=np.hstack((rotation_inv_3.T,t_3)) # stacked [R | t] 3*4 matrix

	def YoloCount_callback(self, data):
		self.data_count = data.count

	def Yolo_callback(self, data):
		self.boundingboxes = data.bounding_boxes
		self.cam_out_num = data.cam_out
		self.image_frame_id = data.frame_id
		
	def Image1_callback(self, data):
		self.image1 = data 
		if self.image_cnt == 0:
			self.pub_image.publish(self.image1)
			
			if self.image_frame_id == "cam1":
				self.image_cnt = 1

	def Image2_callback(self, data):
		self.image2 = data
		# print(data.header)
		if self.image_cnt == 1:
			self.pub_image.publish(self.image2)
			
			if self.image_frame_id == "cam2":
				self.image_cnt = 2

	def Image3_callback(self, data):
		self.image3 = data 
		if self.image_cnt == 2:
			self.pub_image.publish(self.image3)
			
			if self.image_frame_id == "cam3":
				self.image_cnt = 0
	def mission(self):
		self.cam_boundingboxes(self.image_frame_id, self.boundingboxes)

	def cam_boundingboxes(self, cam, bounding_boxes):
		if cam == "cam1":
			cam = 0
		elif cam == "cam2":
			cam = 1
		elif cam == "cam3":
			cam = 2
		if bounding_boxes > 0:
			for i in range(len(bounding_boxes)):
				if bounding_boxes[i].Class == "person":
					self.cam_num = cam
					# print("cam:",cam)
					self.bounding = bounding_boxes
					self.bounding_num = i
					self.person_flag = True 
					self.task()
		else:
			self.bounding = None
			self.boundingboxes = None # 0821 test


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



	def task(self):
		if self.bounding == None:
			pass
		else:
			print("cam_out_num",self.cam_num)
			xmin = self.bounding[self.bounding_num].xmin
			ymin = self.bounding[self.bounding_num].ymin
			xmax = self.bounding[self.bounding_num].xmax
			ymax = self.bounding[self.bounding_num].ymax
			# # Center of box
			xcenter = (xmin+xmax)/2.0
			ycenter = (ymin+ymax)/2.0

			pcl = get_cam_pointcloud(self.soc,self.cam_num)
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
			cv_points = self.lidar_cam_fusion(self.cam_num,pcl_matrix)
			B = np.square((cv_points[0,:]-xcenter))+ np.square((cv_points[1,:]-ycenter))
			# Get index of lidar point for detected object
			index0 = int(np.argmin(B, axis=1))
			print('x:{:.2f} y:{:.2f} distance: {:.2f}'.format(X[index0], Y[index0], distance[index0]))
			if distance[index0]<3:
				if self.cam_num == 0:
					self.alert_calss.person_distance = distance[index0]
					self.alert_calss.shy_away_position = "move_right"
					self.alert_calss.alert_level_cal()
				elif self.cam_num == 1:
					self.alert_calss_1.person_distance = distance[index0]
					self.alert_calss_1.shy_away_position = "move_left"
					self.alert_calss_1.alert_level_cal()
				elif self.cam_num == 2:
					self.alert_calss_2.person_distance = distance[index0]
					self.alert_calss_2.shy_away_position = "move_right"
					self.alert_calss_2.alert_level_cal()

		self.bounding = None							
		print(' ')
# main Alert 
class Alert(threading.Thread):
	def __init__(self, group=None, target=None, name=None, args=(), kwargs=None, verbose=None):
		# Call the Thread class's init function
		super(Alert, self).__init__(group=group, target=self.thread_time_cal,name=name, args=args, kwargs=kwargs,verbose=verbose)
		self.args = args
		self.person_distance = 3
		self.shy_away_position = None
		self.alert_flag = None
		self.alert_response = None

		self.Depth_level = depth_alert()
		self.pub_alert = rospy.Publisher("alert_level", depth_alert, queue_size=10)

	def alert_level_cal(self):
		# print("Distance: %d mm"%self.person_distance)
		if self.person_distance < 1 and self.alert_flag == False:
			self.Depth_level.level = "level_1"
		elif self.person_distance < 1 and self.alert_flag == True:
			self.Depth_level.level = "level_2"
		else :
			self.Depth_level.level = "level_0"
		self.Depth_level.shy_away_position = self.shy_away_position
		self.pub_alert.publish(self.Depth_level)
		print("Depth_level:",self.Depth_level)
	#TODO:If the person leaves the correspondence number
	def thread_time_cal(self):
		count = 0
		while True: 
			try:
				# print("self.level_test:",self.level_test)
				if self.person_distance < 1 and count < 3:
					count += 1
					self.alert_flag = False
					time.sleep(1)
				elif self.person_distance < 1 and count == 3:
					self.alert_flag = True
				elif self.person_distance >= 1:
					count = 0
					self.alert_flag = False
			except:
				print("Other abnormalities in the program")

class Alert_1(threading.Thread):
	def __init__(self, group=None, target=None, name=None, args=(), kwargs=None, verbose=None):
		# Call the Thread class's init function
		super(Alert_1, self).__init__(group=group, target=self.thread_time_cal,name=name, args=args, kwargs=kwargs,verbose=verbose)
		self.args = args
		self.person_distance = 3
		self.shy_away_position = None
		self.alert_flag = None
		self.alert_response = None

		self.Depth_level = depth_alert()
		self.pub_alert = rospy.Publisher("alert_level", depth_alert, queue_size=10)

	def alert_level_cal(self):
		# print("Distance: %d mm"%self.person_distance)
		if self.person_distance < 1 and self.alert_flag == False:
			self.Depth_level.level = "level_1"
		elif self.person_distance < 1 and self.alert_flag == True:
			self.Depth_level.level = "level_2"
		else :
			self.Depth_level.level = "level_0"
		self.Depth_level.shy_away_position = self.shy_away_position
		self.pub_alert.publish(self.Depth_level)
		print("Depth_level:",self.Depth_level)

	def thread_time_cal(self):
		count = 0
		while True: 
			try:
				if self.person_distance < 1 and count < 3:
					count += 1
					self.alert_flag = False
					time.sleep(1)
				elif self.person_distance < 1 and count == 3:
					self.alert_flag = True
				elif self.person_distance >= 1:
					count = 0
					self.alert_flag = False

			except:
				print("Other abnormalities in the program")

class Alert_2(threading.Thread):
	def __init__(self, group=None, target=None, name=None, args=(), kwargs=None, verbose=None):
		# Call the Thread class's init function
		super(Alert_2, self).__init__(group=group, target=self.thread_time_cal,name=name, args=args, kwargs=kwargs,verbose=verbose)
		self.args = args
		self.person_distance = 3
		self.shy_away_position = None
		self.alert_flag = None
		self.alert_response = None

		self.Depth_level = depth_alert()
		self.pub_alert = rospy.Publisher("alert_level", depth_alert, queue_size=10)

	def alert_level_cal(self):
		# print("Distance: %d mm"%self.person_distance)
		if self.person_distance < 1 and self.alert_flag == False:
			self.Depth_level.level = "level_1"
		elif self.person_distance < 1 and self.alert_flag == True:
			self.Depth_level.level = "level_2"
		else :
			self.Depth_level.level = "level_0"
		self.Depth_level.shy_away_position = self.shy_away_position
		self.pub_alert.publish(self.Depth_level)
		print("Depth_level:",self.Depth_level)

	def thread_time_cal(self):
		count = 0
		while True: 
			try:
				if self.person_distance < 1 and count < 3:
					count += 1
					self.alert_flag = False
					time.sleep(1)
				elif self.person_distance < 1 and count == 3:
					self.alert_flag = True
				elif self.person_distance >= 1:
					count = 0
					self.alert_flag = False

			except:
				print("Other abnormalities in the program")

if __name__ == '__main__':
	argv = rospy.myargv()
	rospy.init_node('lidar_yolo_match', anonymous=True)
	rate = rospy.Rate(10) # 10hz

	alert = Alert()
	alert.daemon = True
	alert.start()

	alert_1 = Alert_1()
	alert_1.daemon = True
	alert_1.start()

	alert_2 = Alert_2()
	alert_2.daemon = True
	alert_2.start()

	cal = cal_class(alert,alert_1,alert_2)
	cal.vlp16_socket()
	cal.tranform_cal()
	try:
		while not rospy.is_shutdown():
			cal.mission()
			rate.sleep()
	except KeyboardInterrupt:
		alert.join()
		alert_1.join()
		alert_2.join()
	
	rospy.spin()
