## Official Documentation

  - Checkout the [docs](http://mmwise.github.com/sphero_ros)

### Dependencies

- ROS Indigo
  Go to [this website](http://wiki.ros.org/indigo/Installation/Ubuntu) and follow all the instructions there to install
  your ros environment. Then install catkin. I prefer catkin build to catkin make since the ROS community is migrating away from `make` and it offers its own separation of build space advantages.

- Catkin build

  The best way to install catkin build is via PyPI. Do this in terminal:

  - PyPI Installation

    ```
      sudo pip install -U catkin_tools
    ```

  - Source installation

  Go to the [catkin build website](http://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html) and follow the instructions to install `catkin build`

  Then build catkin

  ```bash
  export ROS_DISTRO=indigo                                 # Set ROS distribution
  mkdir -p ~/catkin_ws/src                       # Create workspace
  cd ~/catkin_ws/src                             # Navigate to source space
  rosinstall_generator --deps ros_tutorials > .rosinstall  # Get list of packages
  wstool update                                            # Checkout all packages
  cd ~/catkin_ws                                 # Navigate to ros workspace root
  catkin init                                              # Initialize workspace
  ```

  - tqdm

  ```bash
    pip install tqdm>=4.11.2
  ```

### Sphero ROS Build

`cd` to the `src` folder of your `catkin_ws` and then clone the hcitool branch of this repo:

```
  cd ~/catkin_ws/src
  git clone https://github.com/service-lab/sphero-ros.git -b hcitool
```

Now cd to the root of your catkin_ws folder and then build:

```bash
  catkin build
```

### Adding Paired Bluetooth Addresses to Launch file

 Pair the bluetooth addresses for other sphero devices that you have using the bluetooth settings tab on your ubuntu distro, e.g. >> `Dash + <search for BlueTooth + Click on the "+" sign to add other bluetooth devices>`.

 Now note the names and addresses of these devices that you have paired with and copy them into the the approporiate tabs in the [multisphero launcher file](/sphero_node/launch/multi_sphero.launch).

### Bringup each robot to the rosgraph

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
