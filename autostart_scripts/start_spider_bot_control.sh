#!/bin/bash

rosws=dev_ws
rospackage=spider_bot

sleep 7

export DISPLAY=:0.0
export LOGFILE=/home/$USER/$rosws/src/$rospackage/autostart_scripts/spider_bot_control.log

source /home/$USER/$rosws/src/$rospackage/autostart_scripts/ROS_CONFIG.txt
source /opt/ros/galactic/setup.bash
source /home/$USER/$rosws/install/local_setup.bash

cd ~/dev_ws/src/spider_bot/spider_bot

while true
do
		echo >>$LOGFILE
		echo "----------------------------------------------" >> $LOGFILE
		date >> $LOGFILE

		echo "Starting python3 spider_bot_control.py" >> $LOGFILE

		python3 -u spider_bot_control.py >> $LOGFILE

		echo "program seems to have stopped" >> $LOGFILE

		date >> $LOGFILE
		sleep 1

done
