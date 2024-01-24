#!/bin/bash

rosws=dev_ws
rospackage=spider_bot

sleep 10

export DISPLAY=:0.0
export LOGFILE=/home/$USER/$rosws/src/$rospackage/autostart_scripts/signaling_server.log

# source /home/$USER/$rosws/src/$rospackage/autostart_scripts/ROS_CONFIG.txt
# source /opt/ros/galactic/setup.bash
# source /home/$USER/$rosws/install/local_setup.bash

cd /home/jetson/web_dev/spider_bot_console/js

while true
do
		echo >>$LOGFILE
		echo "----------------------------------------------" >> $LOGFILE
		date >> $LOGFILE

		echo "Starting signaling_server.js" >> $LOGFILE

		node signaling_server.js >> $LOGFILE

		echo "program seems to have stopped" >> $LOGFILE

		date >> $LOGFILE
		sleep 1

done
