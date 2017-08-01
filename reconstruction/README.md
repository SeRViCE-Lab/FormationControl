## Documentation of the reconstruction package
###### Please refer to the [ReadMe file](http://github.com/SeRViCE-Lab/sphero_ros/blob/master/README.md) in the Official Documentation of the sphero_ros project for more details about running the package.

### Package details
  - This package is designed to find the rotation matrix and translation vector of the camera, and apply 3D reconstruction to the image points of the robots (obtained by subscribing to /locs/ordered) to produce a set of world coordinates. The coordinates are reduced to the x and y components only (z = 0), and they are published to the topic: /pos/world.
  - The package has been tested with a Logitech C920 webcam.
  - To use the package:
    1. Perform a camera calibration beforehand, and store the obtained intrinsic camera matrix and distortion coefficients in the config directory of the package as a yml file. To calibrate the camera use the image_pipeline package that can be cloned from github using [this link](https://github.com/ros-perception/image_pipeline).
    2. Run the package though terminal by using the command
    ```roslaunch reconstruction reconstruction.launch
    ```
    3. The found world points are published as std_msgs::Float64MultiArray messages to the topic: /pos/world. The data is stored in the form of a 1D array with x-y pairs stored successively.
