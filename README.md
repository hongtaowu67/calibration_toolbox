# Camera Calibration Toolbox

To setup the connection of RGBD camera, please refer to [this repository](https://github.com/hongtaowu67/Engineering_Note).

## Intrinsic Calibration
To calibrate instrinsic, follow the instruction [here](http://wiki.ros.org/openni_launch/Tutorials/IntrinsicCalibration). If the camera is an RGBD camera, make sure to calibrate both the RGB and IR camera.

When using the *camera_calibration* ROS package, make sure to check the X, Y, Size, and Skew on the left of the window. Make sure all of them are turned green. Make sure that the camera is moved very closed to the chessboard. The distortion is most obvious when it is close to the chessboard.

## Franka Emika Panda Robot Extrinsic Calibration
