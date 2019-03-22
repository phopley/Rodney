#!/bin/bash
echo "Remap the device serial ports ttyUSBx to rplidar and nano"
echo "Check with commands:"
echo "ls -l /dev|grep ttyUSB"
echo "and"
echo "ls -l /dev|grep ttyACM"
echo " "
echo "sudo cp `rospack find rodney`/scripts/rodney_udev.rules /etc/udev/rules.d"
sudo cp `rospack find rodney`/scripts/rodney_udev.rules /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo " "
sudo service udev reload
sudo service udev restart
echo "finished"

