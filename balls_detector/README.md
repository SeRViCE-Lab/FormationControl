## Documentation of the balls_detector packages
###### Please refer to the [ReadMe file](http://github.com/SeRViCE-Lab/sphero_ros/blob/master/README.md) in the Official Documentation of the sphero_ros project for more details about running the package.

### Package details
  - This package is designed to detect and track the Sphero robots. It has been tested with a Logitech C920 webcam. No testing has been done for Kinect support.
  - The detection, and tracking, is done as explained below:
    1. First, the webcam parameter are modified to reduce exposure and stop autofocus. See modify_camera_param.py
    2. Then, the camera, webcam or Kinect, is used to capture an image, and publish it to a topic. See cam_bridge.py
    3. Then, the Spheros are turned off (LEDs), each on is turned on(LEDs), one at a time, detected, and then turned off(LEDs). Once this has been done to all the Spheros, the Spheros are turned back on (LEDs). The recorded centroid data is then published repeatedly. See tagger.py
    3. Then, a blob detection algorithm is used to find the pixel coordinates of the Sphero robots. The coordinates are published in a std_msgs/Float64MultiArray format to a topic. See balls_detector_node.cpp
    4. Finally, the pixels coordinates are rearranged, to allow for tracking of the Sphero robots, using a bijective nearest neighbor rule. See tracker.py

###### TODD:
  - Kinect has not been tested with this package. In cam_bridge.py, there is an option to use Kinect, but it has not been tested out. Moreover, the detection algorithm, itself, has been designed to work with modified camera parameter, which is only done for the webcam.
    If Kinect is to be used, install libraries and dependencies by following the instructions on [openkinect](http://openkinect.org).
