## Official Documentation

  - Checkout the [docs](http://mmwise.github.com/sphero_ros)

### Dependencies

* ROS
 * Go to [the ROS website](http://wiki.ros.org/indigo/Installation/Ubuntu) and follow all the instructions there to install
  your ros environment.
 * Then install catkin.
  * Catkin build:  The best way to install catkin build is via PyPI. Do this in terminal:  
    ```
      sudo pip install -U catkin_tools
    ```
 * Build your workspace

  ```bash
  export ROS_DISTRO=`rosversion -d`              # Set ROS distribution
  mkdir -p ~/catkin_ws/src                       # Create workspace
  cd ~/catkin_ws/src                             # Navigate to source space
  cd ~/catkin_ws                                 # Navigate to ros workspace root
  catkin init                                              # Initialize workspace
  ```
* Install opencv 2.4

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

 Pair the bluetooth for allyour sphero devices using the bluetooth settings tab on your Linux distro, e.g. >> `Dash + <search for BlueTooth + Click on the "+" sign to add other bluetooth devices>`.

 Now note the names and addresses of these devices that you have paired and copy them into the the appropriate tabs in the [multisphero launcher file](/sphero_node/launch/multi_sphero.launch).

### Bringup each robot into ros

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

Install Kinect libraries and dependencies by following the instructions on [openkinect](http://openkinect.org).

Then launch the balls detector node like so:

````bash
  roslaunch balls_detector kinect.launch
```
