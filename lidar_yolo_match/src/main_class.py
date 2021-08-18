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
from velodyne_capture_v3 import init_velo_socket, get_pointcloud, get_cam_pointcloud
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Circle
from collisionNew import Human, collision_detection
# yolo_detection
from yolo_detection.msg import ROI_array
from yolo_detection.msg import ROI
from yolo_detection.msg import cam_output
# from yolo_detection.msg import depth_alert

from darknet_ros_msgs.msg import BoundingBox
from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import ObjectCount

from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray

from sensor_msgs.msg import Image 
# Sub-execution work function
import threading

from lidar_yolo_match.msg import depth_alert
from lidar_yolo_match.srv import alert_output, alert_outputResponse
from lidar_yolo_match.srv import TimdaMode, TimdaModeResponse

class cal_class:
	def __init__(self, alert_calss):
		self.obj_num = 0
		self.boundingboxes = None
		self.cam_out_num = 0
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
		self.alert_calss = alert_calss
		# self.sub = rospy.Subscriber("chatter",String,self.callback)
		self.sub_bouding = rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes,self.Yolo_callback)
		self.sub_YOLOCount = rospy.Subscriber("/darknet_ros/found_object",ObjectCount,self.YoloCount_callback)
		self.sub_Image1 = rospy.Subscriber("/camera1/usb_cam1/image_raw",Image,self.Image1_callback)
		self.sub_Image2 = rospy.Subscriber("/camera2/usb_cam2/image_raw",Image,self.Image2_callback)
		self.sub_Image3 = rospy.Subscriber("/camera3/usb_cam3/image_raw",Image,self.Image3_callback)
		self.pub_cam_num  =  rospy.Publisher("/cam_num", cam_output, queue_size=10)
		self.pub_image = rospy.Publisher("/usb_cam/image_raw", Image, queue_size=None)
		
	def vlp16_socket(self):
		# Set up sockets:
		HOST = "192.168.1.201"
		PORT = 2368
		self.soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.soc.bind(('', PORT))

	def tranform_cal(self):
		self.camera_matrix = np.matrix([[769.534729,0.000000,653.262129],[0.000000,788.671204,360.453779],[0.000000,0.000000,1.000000]])
		avg_transformation = np.matrix([[0.99911,0.0119529,0.040449,0.0160659],[-0.0123006,0.999889,0.00835869,-0.0894833],[-0.0403446,-0.00884879,0.999147,0.0650598],[0,0,0,1]])
		final_rotation = np.matrix([[-0.029497,-0.998977,-0.0342695],[-0.0972901,0.0369909,-0.994568],[0.994819,-0.0260027,-0.0982818]])
		rotation_inv = inv(final_rotation)
		avg_transformation_inv = inv(avg_transformation)
		t=np.array([[0],[0],[0]]) 
		t=np.array([[-0.07295466],[0.02105128],[-0.08205254]]) 
		# t=np.array([[-0.01452749],[0.08985711],[-0.06490614]]) 
		# h=np.hstack((rotation_inv.T,t))                           # stacked [R | t] 3*4 matrix
		self.h=np.hstack((rotation_inv.T,t))

	def YoloCount_callback(self, data):
		self.data_count = data.count

	def Yolo_callback(self, data):
		self.boundingboxes = data.bounding_boxes
		self.cam_out_num = data.cam_out
		self.obj_num = len((data.bounding_boxes))
		# self.cam_change_flag = self.cam_boundingboxes(self.cam_out_num,self.boundingboxes)
		
	def Image1_callback(self, data):
		self.image1 = data 
		if self.image_cnt == 0:
			self.pub_image.publish(self.image1)
			self.pub_cam_num.publish(0)

			if self.cam_out_num == 0:
				self.cam_change_flag = self.cam_boundingboxes(self.cam_out_num, self.boundingboxes)
				self.image_cnt = 1

	def Image2_callback(self, data):
		self.image2 = data 
		if self.image_cnt == 1:
			self.pub_image.publish(self.image2)
			self.pub_cam_num.publish(1)

			if self.cam_out_num == 1:
				self.cam_change_flag = self.cam_boundingboxes(self.cam_out_num, self.boundingboxes)
				self.image_cnt = 2

	def Image3_callback(self, data):
		self.image3 = data 
		if self.image_cnt == 2:
			self.pub_image.publish(self.image3)
			self.pub_cam_num.publish(2)

			if self.cam_out_num == 2:
				self.cam_change_flag = self.cam_boundingboxes(self.cam_out_num, self.boundingboxes)
				self.image_cnt = 0

	def cam_boundingboxes(self, cam, bounding_boxes):
		if self.boundingboxes > 0:
			for i in range(len(self.boundingboxes)):
				if self.boundingboxes[i].Class == "person":
					self.cam_num = self.cam_out_num
					self.bounding = self.boundingboxes
					self.bounding_num = i
					# self.match_task()
			return True
		else:
			self.bounding = None
			return False
	def match_task(self):
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
		# pcl_matrix = np.hstack([pcl_matrix,W1])
		# print("ducccccck",pcl_matrix)
		#-------------
		F = np.matmul((self.h),(pcl_matrix))
		cv_points = np.matmul((self.camera_matrix),(F))/F[2,:]

		imPoints=self.h.dot(pcl_matrix)        # transforming points from world frame to camera frame
		imPoints=self.camera_matrix.dot(imPoints)        # projecting points to image plane
		imPoints=imPoints/imPoints[2,:] 

		B = np.square((cv_points[0,:]-xcenter))+ np.square((cv_points[1,:]-ycenter))
		# Get index of lidar point for detected object
		index0 = int(np.argmin(B, axis=1))
		# TODO: Distance conversion and testing
		print('x:{:.2f} y:{:.2f} distance: {:.2f}'.format(X[index0], Y[index0], distance[index0]))
		self.alert_calss.person_distance = distance[index0]
		self.alert_calss.alert_level_cal()
		# self.alert_calss.alert_response = self.alert_calss.alert_client_to_timda_server(self.alert_calss.Depth_level)						
		print(' ')

	def task(self):
		if self.cam_change_flag == True:
			self.cam_change_flag = False
			if self.bounding == None:
				pass
			else:
				# for i in range(len(self.bounding)):
				# 	if self.bounding[i].Class == "person":
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
				# pcl_matrix = np.hstack([pcl_matrix,W1])
				# print("ducccccck",pcl_matrix)
				#-------------
				F = np.matmul((self.h),(pcl_matrix))
				cv_points = np.matmul((self.camera_matrix),(F))/F[2,:]

				imPoints=self.h.dot(pcl_matrix)        # transforming points from world frame to camera frame
				imPoints=self.camera_matrix.dot(imPoints)        # projecting points to image plane
				imPoints=imPoints/imPoints[2,:] 

				B = np.square((cv_points[0,:]-xcenter))+ np.square((cv_points[1,:]-ycenter))
				# Get index of lidar point for detected object
				index0 = int(np.argmin(B, axis=1))
				# TODO: Distance conversion and testing
				print('x:{:.2f} y:{:.2f} distance: {:.2f}'.format(X[index0], Y[index0], distance[index0]))
				self.alert_calss.person_distance = distance[index0]
				self.alert_calss.alert_level_cal()
				# self.alert_calss.alert_response = self.alert_calss.alert_client_to_timda_server(self.alert_calss.Depth_level)
			self.bounding = None							
			print(' ')

class Alert(threading.Thread):
	def __init__(self, group=None, target=None, name=None, args=(), kwargs=None, verbose=None):
		# Call the Thread class's init function
		super(Alert, self).__init__(group=group, target=self.thread_time_cal,
                               name=name, args=args, kwargs=kwargs,
                               verbose=verbose)
		self.args = args
		self.person_distance = 3
		self.alert_flag = None
		self.alert_response = None

		self.Depth_level = depth_alert()
		self.pub_alert = rospy.Publisher("alert_level", depth_alert, queue_size=10)
	
	# example  client  Person detection warning request
	def alert_client_to_timda_server(self, req):
		rospy.wait_for_service('TIMDA_SERVER')
		print("stay alert input")
		try:
			alert = rospy.ServiceProxy('TIMDA_SERVER', TimdaMode)
			alert_resp = alert(req)
			return alert_resp
		except rospy.ServiceException as e:
			print("Service call failed: %s"%e)

	def alert_level_cal(self):
		# print("Distance: %d mm"%self.person_distance)
		if self.person_distance < 1 and self.alert_flag == False:
			self.Depth_level = "level_1"
		elif self.person_distance < 1 and self.alert_flag == True:
			self.Depth_level = "level_2"
		else :
			self.Depth_level = "level_0"

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
				# test !!
				# print('thread_time_cal')
				# time.sleep(2)
			except:
				print("Other abnormalities in the program")

if __name__ == '__main__':
	argv = rospy.myargv()
	rospy.init_node('lidar_yolo_match', anonymous=True)
	rate = rospy.Rate(10) # 10hz

	alert = Alert()
	alert.daemon = True
	alert.start()

	cal = cal_class(alert)
	cal.vlp16_socket()
	cal.tranform_cal()
	# cal.matrix()
	# cal.Transpose()
	try:
		while not rospy.is_shutdown():
			cal.task()
			rate.sleep()
	except KeyboardInterrupt:
		alert.join()
	
	rospy.spin()
