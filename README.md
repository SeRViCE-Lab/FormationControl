## Official Documentation

  - Checkout the [docs](http://mmwise.github.com/sphero_ros)

### Dependencies

- ROS

  Go to [this website](http://wiki.ros.org/indigo/Installation/Ubuntu) and follow all the instructions there to install
  your ros environment.

  Then install catkin. I prefer catkin build to catkin make since it offers its own separation of build space advantages.

- Catkin build

  The best way to install catkin build is via PyPI. Do this in terminal:

    ```
      sudo pip install -U catkin_tools
    ```

- Build your workspace

  ```bash
  export ROS_DISTRO=`rosversion -d`              # Set ROS distribution
  mkdir -p ~/catkin_ws/src                       # Create workspace
  cd ~/catkin_ws/src                             # Navigate to source space
  # rosinstall_generator --deps ros_tutorials > .rosinstall  # Get list of packages
  # wstool update                                            # Checkout all packages
  cd ~/catkin_ws                                 # Navigate to ros workspace root
  catkin init                                              # Initialize workspace
  ```


### Sphero ROS Build

`cd` to the `src` folder of your `catkin_ws` and then clone the `master` branch of this repo:

```
  cd ~/catkin_ws/src
  git clone https://github.com/service-lab/sphero-ros.git -b master
```

Now cd to the root of your catkin_ws folder and then build:

```bash
  cd ~/catkin_ws
  catkin build
```

### Adding Paired Bluetooth Addresses to Launch file

 Pair the bluetooth addresses for other sphero devices that you have using the bluetooth settings tab on your ubuntu distro, e.g. >> `Dash + <search for BlueTooth + Click on the "+" sign to add other bluetooth devices>`.

 Now note the names and addresses of these devices that you have paired with and copy them into the the appropriate tabs in the [multisphero launcher file](/sphero_node/launch/multi_sphero.launch).

### Bringup each robot into roscore

```bash
  roslaunch sphero_node multi_sphero.launch
```

This should bring up each robot to the ros graph with each robot ready to receive commands on any of the topics

```bash
/sphero_xxx/cmd_vel
/sphero_xxx/collision
/sphero_xxx/disable_stabilization
/sphero_xxx/imu
/sphero_xxx/odom
/sphero_xxx/set_angular_velocity
/sphero_xxx/set_back_led
/sphero_xxx/set_color
/sphero_xxx/set_heading
```

### Example Command Velocity Code

```bash
  rosrun sphero_run sphero_move.py
```


### Vision based tracking of balls

Detection and tracking of the Sphero robots is done in the balls_detector package.
The package is designed to support a Kinect or a generic Webcam. However, at this stage, the Kinect camera has not been tested nor fully supported. We used the Logitech C920 webcam to test the package, to reproduce our results, please use a webcam.
  If you decide to use Kinect, install Kinect libraries and dependencies by following the instructions on [openkinect](http://openkinect.org).

To use the balls_detector package with a webcam, run the nodes manually in terminal by follow the steps below:
  1. Connect to the spheros and set the color of the LEDs to white at maximum brightness (255,255,255).
  2. Install v4l if not installed. This will allow us to modify the camera parameters. To install v4l, in a terminal window, run the following command:
  ```bash
  sudo apt-install v4l-utils
  ```
  3. In a new terminal window, change directory to your catkin workspace, then roslaunch the camera.launch file in balls_detector as such:
  ```bash
  cd ~/catkin_ws/
  roslaunch balls_detector camera.launch
  ```
  4. When the launch file starts, a pop-up window should become visible. That window should be dark and only the Sphero robots should appear in it. The detection algorithm should display yellow circles on the Spheros once they are detected. If that does not happen, then the detection parameters need to be modified. To modify camera parameters, change them manually in /balls_detector/launch/detect_param.yaml
  Also, when you should have three new topics, namely: "/cam/image/rgb", "/locs/detected", and "/locs/ordered". The three topics should be publishing data, with the first publishing image data, the second publishing a list of the detected centroids, and the third publishing an ordered list of the centroids such that the first centroids belongs to the first robot detected when the detection algorithm was initialized, the second robot being the second one detected at the start, and so on.
  To check these topics, in a new terminal run:
  ```bash
  rostopic list
  rostopic echo /locs/detected
  ```
  Change the topic name to any of the other topics to display their output.

Notes:
- The detection parameters are dependent on the height of the camera and the orientation of the robots. Modify the parameters based on camera position if necessary. When we tested it, the camera was placed on a tripod, and placed at a hight of 1.65 meters with the lens pointing downwards.
- In the first iteration, do not have any other objects in the frame, have only the Sphero robots present and colored white.

###### TODO:
Kinect support need to be extended.  
