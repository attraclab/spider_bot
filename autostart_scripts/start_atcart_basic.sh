#!/bin/bash

rosws=dev_ws
rospackage=spider_bot

sleep 5

export DISPLAY=:0.0
export LOGFILE=/home/$USER/$rosws/src/$rospackage/autostart_scripts/atcart_basic.log

source /home/$USER/$rosws/src/$rospackage/autostart_scripts/ROS_CONFIG.txt
source /opt/ros/galactic/setup.bash
source /home/$USER/$rosws/install/local_setup.bash


while true
do
		echo >>$LOGFILE
		echo "----------------------------------------------" >> $LOGFILE
		date >> $LOGFILE

		echo "Starting jmoab_ros2 atcart_basic" >> $LOGFILE

		ros2 run jmoab_ros2 atcart_basic >> $LOGFILE

		echo "program seems to have stopped" >> $LOGFILE

		date >> $LOGFILE
		sleep 1

done
