#/bin/bash
$(rostopic pub /sphero1/cmd_vel geometry_msgs/Twist -r 10 '[100, 0, 0]' '[0, 0, 0]')
