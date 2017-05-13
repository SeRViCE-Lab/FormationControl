#! /bin/bash
x1=$(rostopic pub /sphero_rgo/cmd_vel geometry_msgs/Twist -r 10 '[60, 0, 0]' '[0, 0, 0]')
x2=$(rostopic pub /sphero_ypp/cmd_vel geometry_msgs/Twist -r 10 '[60, 0, 0]' '[0, 0, 0]')
x3=$(rostopic pub /sphero_pob/cmd_vel geometry_msgs/Twist -r 10 '[60, 0, 0]' '[0, 0, 0]')
x4=$(rostopic pub /sphero_www/cmd_vel geometry_msgs/Twist -r 10 '[60, 0, 0]' '[0, 0, 0]')
x5=$(rostopic pub /sphero_rpg/cmd_vel geometry_msgs/Twist -r 10 '[60, 0, 0]' '[0, 0, 0]')
x6=$(rostopic pub /sphero_rwp/cmd_vel geometry_msgs/Twist -r 10 '[60, 0, 0]' '[0, 0, 0]')
x7=$(rostopic pub /sphero_wwb/cmd_vel geometry_msgs/Twist -r 10 '[60, 0, 0]' '[0, 0, 0]')
x8=$(rostopic pub /sphero_gbr/cmd_vel geometry_msgs/Twist -r 10 '[60, 0, 0]' '[0, 0, 0]')
