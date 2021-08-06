#!/usr/bin/env python3
import sys
import rospy
import os
import threading
import numpy as np


import math
import enum
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

image1 = None
image2 = None
image3 = None
cnt = 0
data_count = 0
ROIarray = None

cam_out_num = None

class bounding_boxes():
    def __init__(self,probability,xmin,ymin,xmax,ymax,id_name,Class_name):
        self.probability = probability
        self.xmin = xmin
        self.ymin = ymin
        self.xmax = xmax
        self.ymax = ymax
        self.id_name = str(id_name)
        self.Class_name = str(Class_name)

# boxes = bounding_boxes(0,0,0,0,0,0,0)
# YOLO V4 輸入
i = 0
def Yolo_callback(data):
    global obj_num, ROIarray, cam_out_num

    boxes = bounding_boxes(0,0,0,0,0,0,0)
    #global probability,xmin,ymin,xmax,ymax,id_name,Class_name
    cam_out_num = data.cam_out
    obj_num = len((data.bounding_boxes))
    if obj_num == 0:
        pass
            # print("No Object Found!")
            # print("change method to Realsense!")
    else:
        #i = obj_num-1
        List = []
        for i in range(len(data.bounding_boxes)):
            if data.bounding_boxes[i].Class == "person":
                boxes.probability = data.bounding_boxes[i].probability
                boxes.xmin = data.bounding_boxes[i].xmin
                boxes.ymin = data.bounding_boxes[i].ymin
                boxes.xmax = data.bounding_boxes[i].xmax
                boxes.ymax = data.bounding_boxes[i].ymax
                boxes.id_name = data.bounding_boxes[i].id
                boxes.Class_name = data.bounding_boxes[i].Class
                
                center_x  = (boxes.xmax+boxes.xmin)/2
                center_y  = (boxes.ymax+boxes.ymin)/2

                ROI_data = ROI()
                ROI_data.probability = boxes.probability
                ROI_data.object_name= boxes.Class_name
                ROI_data.id = boxes.id_name
                ROI_data.x = center_x
                ROI_data.y = center_y
                ROIarray = ROI_array()
                List.append(ROI_data)
                ROIarray.ROI_list = List
            
        print("ROI_array:",ROIarray)
        # pub.publish(ROIarray)

            
def YoloCount_callback(data):
    global data_count
    data_count = data.count

def Image1_callback(data):
    global cnt, image1, image2, data_count, ROIarray, cam_out_num
    image1 = data 
    # pub_image.publish(image1)
    if cnt == 0:
        pub_image.publish(image1)

        cam_num = cam_output()
        cam_num = 0
        pub_cam_num.publish(cam_num)

        # rospy.loginfo(rospy.get_caller_id() + "image1")
      
        
        if cam_out_num == 0:
            if data_count > 0:
                # rospy.loginfo(rospy.get_caller_id() + "data_count %d",data_count)
                pub.publish(ROIarray)
                ROIarray = None
            cnt = 1

def Image2_callback(data):
    global cnt, image1, image2, data_count, ROIarray, cam_out_num
    image2 = data 
    # pub_image.publish(image2)
    if cnt == 1:
        pub_image.publish(image2)

        cam_num = cam_output()
        cam_num = 1
        pub_cam_num.publish(cam_num)

        # rospy.loginfo(rospy.get_caller_id() + "image2")
       
        if cam_out_num == 1:
            if data_count > 0:
                # rospy.loginfo(rospy.get_caller_id() + "cam_out_num %d",cam_out_num)
                # rospy.loginfo(rospy.get_caller_id() + "data_count %d",data_count)
                pub.publish(ROIarray)
                ROIarray = None
            cnt = 0


if __name__ == '__main__':
    #global boxes
    argv = rospy.myargv()
    rospy.init_node('yolo_boundingboxes', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes,Yolo_callback)
    rospy.Subscriber("/darknet_ros/found_object",ObjectCount,YoloCount_callback)

    rospy.Subscriber("/camera1/usb_cam1/image_raw",Image,Image1_callback)
    rospy.Subscriber("/camera2/usb_cam2/image_raw",Image,Image2_callback)
    # rospy.Subscriber("/camera3/usb_cam3/image_raw",Image,Image3_callback)
 
    pub = rospy.Publisher("/obj_position", ROI_array, queue_size=10)

    pub_cam_num  =  rospy.Publisher("/cam_num", cam_output, queue_size=10)
    pub_image = rospy.Publisher("/usb_cam/image_raw", Image, queue_size=10)
    while not rospy.is_shutdown():
        # strategy()
        # os.system("clear")
        rate.sleep()
    rospy.spin()
