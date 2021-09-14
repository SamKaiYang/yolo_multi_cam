# Yolo v4使用在ROS架構方法

- yolov4 darknet下載
- 參數調整解釋
- usb_cam匯入
- 實際測試
## Prepare
- 軟體環境：Ubuntu18.04,ROS-melodic,OpenCV3.2(代碼中僅能使用含3.2以下)
如果使用Ubuntu16.04 請將下述"melodic"改為"kinetic"即可
## How to used
### build
```
cd <catkin_workspace>/src
#git clone https://github.com/leggedrobotics/darknet_ros # up to v3
git clone https://github.com/tom13133/darknet_ros # up to v4
cd darknet_ros/ && git submodule update --init --recursive
cd ~/catkin_workspace
# before build, check (-O3 -gencode arch=compute_<version>,code=sm_<version>) part in darknet_ros/darknet_ros/CMakeLists.txt if you use CUDA
# ex) 75 for GTX1650 and RTX2060, 86 for RTX3080, 62 for Xavier 
catkin_make

#download weight
cd <catkin_workspace>/src/darknet_ros/darknet_ros/yolo_network_config/weights
# yolo v4
wget https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v3_optimal/yolov4.weights
# yolo v4 tiny
wget https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v4_pre/yolov4-tiny.weights


git clone https://github.com/ros-drivers/velodyne.git
git clone https://github.com/bosch-ros-pkg/usb_cam.git
sudo apt-get install ros-melodic-image-view
sudo apt-get install ros-melodic-usb-cam
cd ../..
catkin_make
source devel/setup.bash
```

### 實際運行
```
cd 自己的workspace/
source devel/setup.bash
roslaunch usb_cam usb_cam-test_multi.launch
```
git clone https://github.com/ros-drivers/velodyne.git
git clone https://github.com/bosch-ros-pkg/usb_cam.git
git clone https://github.com/UT18-Senior-Design/Object-Detection-and-Calibrations.git
### 使用web cam  
```
cd darknet_ros/config
sudo gedit ros.yaml
修改為
  camera_reading:
    topic: /usb_cam/image_raw

cd darknet_ros/launch
sudo gedit darknet_ros.launch 
第6.23行 修改為
"/camera/rgb/image_raw"
```

- 在開啟一個終端
```
cd 自己的workspace/
source devel/setup.bash
roslaunch darknet_ros yolo_v4_tiny.launch
```
- 在開啟一個終端,multi cam yolo detection
```
cd 自己的workspace/
source devel/setup.bash
rosrun yolo_detection multi_cam_yolo.py
```



# 0803 how to use 
```
roslaunch usb_cam usb_cam-test_multi.launch
roslaunch lidar_yolo_match multi_cam_yolo_lidar.launch
```