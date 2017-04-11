#! /bin/bash
x1=$(rostopic pub /sphero1/cmd_vel geometry_msgs/Twist -r 10 '[100, 0, 0]' '[0, 0, 0]')
x2=$(rostopic pub /sphero2/cmd_vel geometry_msgs/Twist -r 10 '[100, 0, 0]' '[0, 0, 0]')
x3=$(rostopic pub /sphero3/cmd_vel geometry_msgs/Twist -r 10 '[100, 0, 0]' '[0, 0, 0]')
x4=$(rostopic pub /sphero0/cmd_vel geometry_msgs/Twist -r 10 '[100, 0, 0]' '[0, 0, 0]')

$x1;
$x2;
$x3;
$x4
