## ####### 首先设置Settings.yaml中的路径 ####### ##

# 1. 简介

calibration包是利用手眼标定估计相机上Motion Capture的marker刚体和相机坐标系之间的变换Tmc(Camera中的点的坐标变换到Marker body坐标系中)的工具包，涵盖了4个独立程序：

1. **collect_intrinsic_data:** 

   a. 用于采集相机内参标定图像（按 's'/'S'键保存）；

   b. 需要设置`Image_Topic`，以及输出保存路径；

   c. 输出为`保存的图片`以及形成的图片列表`image_list.yaml`

2. **calibrate_intrinsic:** 

   a. 基于opencv 的sample 修改成的相机内参标定程序；

   b. 需要设置标定板的参数，以及图片列表路径；

   c. 输出为内参标定结果`calibrate_result.yaml`

3. collect_hand_eye_data: 

   a. 将相机和Motion Capure的data stream进行时间戳对齐，并保存('s'/'S'按键)用于手眼标定；

   b. 需要设置`Image_Topic`，Motion Capture发布的位姿`Marker_Pose_Topic`，以及保存路径；

   c. 输出为`保存的图片`以及形成的图片列表`image_list.yaml`，以及相应的Marker姿态`marker_poses.yaml`

4. **calibrate_hand_eye:**

   a. 采用多种方法进行手眼标定计算，得到Tmc， 并统计Twb = Twm . Tmc. Tcb（标定板坐标系在MC的世界坐标系中的位姿12个元素分别的误差统计（越小越好）

   b. 设置相机内参文件路径`calibrate_result.yaml`， 以及手眼标定的图片列表`image_list.yaml`， 以及相应的Marker姿态`marker_poses.yaml`

   c. 输出为手眼标定结果及误差统计，保存在`handeye_result.yaml`中

# 2. 编译安装

0. 代码需要在catkin工作空间中

1. 下载calibration代码

```shell
cd ~/catkin_ws/src
git clone https://gitee.com/xubinlin/calibration/tree/master
```

2. calibration依赖的OpenCV的版本需要与cv_bridge依赖的版本相同；如果相采用自己安装的OpenCV，则需要重新编译cv_bridge；如果想采用原先ROS中的cv_bridge版本，则calibration的OpenCV需要指向安装ROS时自带的OpenCV，修改`CMakeLists.txt`。我采用前者，重新编译cv_bridge:

```shell
cd ~/catkin_ws/src
git clone https://github.com/ros-perception/vision_opencv.git
cd vision_opencv
git branch -a
git checkout origin/melodic 
export OpenCV_DIR=/usr/local/share/OpenCV
```

3. 编译cv_bridge和calibration

```shell
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
```



# 3. 使用

1. 使用的时候主要是修改Settings.yaml中的Topic名称，标定板参数，以及各个程序的输入、输出文件名及路径；

2. 不一定要使用这个包的内参标定程序，但是手眼标定的时候需要有内参标定文件`calibrate_result.yaml`，需要按照相应的格式手动更改数据；

3. 代码运行指令：

   ```shell
   ## 内参
   roslaunch kinect2_bridge kinect2_bridge.launch (for kinect sensor)
   roslaunch realsense2_camera rs_rgbd.launch (for realsense sensor)
   rosrun calibration collect_intrinsic_data <PATH_TO_SRC>/Settings.yaml
   
   rosrun calibration calibrate_intrinsic <PATH_TO_SRC>/Settings.yaml
   
   ##手眼标定
   roslaunch kinect2_bridge kinect2_bridge.launch (for kinect sensor)
   roslaunch realsense2_camera rs_rgbd.launch (for realsense sensor)
   roslaunch vrpn_client_ros sample.launch
   rosrun calibration collect_hand_eye_data <PATH_TO_SRC>/Settings.yaml
   
   rosrun calibration calibrate_hand_eye
   ```