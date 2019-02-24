#!/bin/bash
echo "delete remap rules for rplidar, nano and teensy"
echo "sudo rm /etc/udev/rules.d/rodney_udev.rules"
sudo rm /etc/udev/rules.d/rodney_udev.rules
echo " "
echo "Restarting udev"
echo " "
sudo service udev reload
sudo service udev restart
echo "finished"
