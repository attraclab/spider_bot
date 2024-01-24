#!/bin/bash

rosws=dev_ws
rospackage=spider_bot

sleep 20

export DISPLAY=:0.0
export LOGFILE=/home/$USER/$rosws/src/$rospackage/autostart_scripts/browser.log

while true
do
	echo "Starting Console view on browser" >> $LOGFILE

	if ping -q -c 1 -W 1 google.com >/dev/null; then
		echo "The network is up" >> $LOGFILE
		sleep 1
		
		## If we make sure the browser is working, we can use --headless mode
		## this will have browser running in background and no worry of refresh issue

		# firefox --kiosk http://localhost/spider_bot_console/offer.html >> $LOGFILE
		firefox --headless http://localhost/spider_bot_console/offer.html >> $LOGFILE


		echo "Console view seems to stop working..." >> $LOGFILE
		date >> $LOGFILE
		echo "----------------------------------------------" >> $LOGFILE
		sleep 1
	else
	  echo "The network is down, keep checking..." >> $LOGFILE
	  sleep 1
	fi

done
