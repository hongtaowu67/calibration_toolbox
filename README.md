# Camera Calibration Toolbox
A toolbox for hand-eye calibration for camera on a manipulator.
It has been tested on UR5 and Franka Emika Panda robot.
Currently, it supports calibration on the Franka Emika Panda robot.
To setup the connection of RGBD camera, please refer to [this repository](https://github.com/hongtaowu67/Engineering_Note).

<p align="center">
  <img src="./doc/thumbnail0.jpg" width="640"/>
</p>


## Installation
* [libfranka](https://frankaemika.github.io/docs/installation_linux.html) & [franka_ros](https://frankaemika.github.io/docs/installation_linux.html)
* [MoveIt](http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/getting_started/getting_started.html#install-ros-and-catkin)
* [openni_camera](http://wiki.ros.org/openni_camera) & [openni_launch](https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=&ved=2ahUKEwj3_6XqvfDvAhXXR30KHdprC3cQFjAAegQIChAE&url=http%3A%2F%2Fwiki.ros.org%2Fopenni_launch&usg=AOvVaw18FvTTmJ3VTTl4SuD4bV0d): for PrimeSense camera
* [librealsense](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md) & [realsense-ros](https://github.com/IntelRealSense/realsense-ros): for RealSense camera
* OpenCV
```
pip install opencv-python
```

## Intrinsic Calibration
To calibrate instrinsic, follow the instruction [here](http://wiki.ros.org/openni_launch/Tutorials/IntrinsicCalibration). If the camera is an RGBD camera, make sure to calibrate both the RGB and IR camera.

When using the *camera_calibration* ROS package, make sure to check the X, Y, Size, and Skew on the left of the window. Make sure all of them are turned green. Move the camera very closed to the chessboard. The distortion is most obvious when it is close to the chessboard.

## Franka Emika Panda Robot Extrinsic Calibration
This repository solves AXXB problem to get the extrinsic calibration problem. 
The whole process goes through 2 steps: collecting data and solving AXXB. 
Now only eye-on-hand configuration is implemented for the data collection. 
But for solving AXXB, both eye-on-hand and hand-on-eye are implemented.

There are 3 options of calibration:
- ```EH```: eye-on-hand, calibrate the pose of the camera frame in the end-effector frame
- ```EBME```: eye-on-base, calibrate the pose of the marker in the end-effector frame
- ```EBCB```: eye-on-base, calibrate the pose of the camera in the base frame

### Data Collection (for EH with Panda robot)
In the data collection part, the robot moves to different configurations and collects the transformation from the hand (end effector) to base and the corresponding transformation from camera to chessboard.

1. Print the calibration chessboard in **doc/chessboard_A4.pdf** (**doc/chessboard_letter.pdf**) if you are using A4 (letter) paper. The size of each square is 29mm in chessboard_A4.pdf and 25.6mm in chessboard_letter.pdf.
2. Attach the chessboard onto a flat rigid plate and fix it on the table.
3. Move the robot to at least 15 configurations. Make sure in each configuration, the camera can see the whole chessboard and the chessboard is LARGE. Because if the chessboard is too small, the estimation of pose will be inaccurate.
4. In **src/main_collector.cpp**, specify the data saving directory and the configurations you collected.
5. In **src/main_collector.cpp**, specify the image topic of the camera. For PrimeSense 1.09, the rgb topic is "/camera/rgb/image_raw".
6. In **src/main_collector.cpp**, specify the base and end-effector link.
7. Initialize the camera. To initialize PrimeSense 1.09,
```
roslaunch openni2_launch openni2.launch
```
8. Launch the Panda robot
```
roslaunch panda_moveit_config panda_control_moveit_rviz.launch robot_ip:=<robot_ip> load_gripper:=<true/false>
```
9. Collect the data. Currently only the panda robot is implemented
```
rosrun calibration_toolbox main_collector
```
The robot will move through the configurations and collect the data. After the collection process is finished, each image is paired with a robot pose file. The image filename is "x_img.png"/; the robot pose filename is "x_robotpose.txt".

### Chessboard Detection
This repo uses OpenCV to find the chessboard pattern and get the pose of the chessboard in the camera frame.

1. Specify the chessboard pattern, camera info yaml, and data directory in **src/calibration_toolbox/chessboard_detection.py**.
2. Run the chessboard detector
```
python chessboard_detector.py
```
The chessboard poses are saved in the data directory. marker poses are saved in "x_markerpose.txt".

### Solve AXXB to get the eye-to-hand transformation
Use the Park & Martin's method to solve AXXB problem.
1. Specify the __data directory__ and __calibration option__ in **src/calibration_toolbox/main_calibrate.py**
2. Solve the AXXB problem
```
python main_calibrate.py
```
The calibrated transformation will be saved in the data directory as ```pose.txt``` and ```pose.yaml```. 
Make sure to check each of the check pose in the terminal.
If the calibration is successful, they should be very close to each other.

## TODO
- [ ] Test different calibration options