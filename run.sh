#/bin/bash
$(rostopic pub /sphero1/cmd_vel geometry_msgs/Twt -r 10 '[100, 0, 0]' '[0, 0, 0]')
