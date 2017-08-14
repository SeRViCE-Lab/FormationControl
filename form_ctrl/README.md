## Formation Control Package

- This package contains the formation control
- It subscribes to /locs/ordered, the ordered image location points of the robots, and to /pos/world, the ordered world location points of the robots, runs the formation control code on the data (heading estimation, and formation control), produces the necessary velocity and heading data in m/s and degrees, then publishes this data as geometry_msgs::Twist messages to each robot's topic (ex: /sphero_xxx/cmd_vel). 
