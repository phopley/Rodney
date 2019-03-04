#!/bin/bash
# script to get git repositories for the Rodney Robot project, with the exception of the rodney package.
# run the script (./clonepackages) from a clone of the rodney package
cd ../..
git clone https://github.com/phopley/face_recognition.git
git clone https://github.com/phopley/face_recognition_msgs.git
git clone https://github.com/phopley/head_control.git
git clone https://github.com/phopley/joystick.git
git clone https://github.com/phopley/pan_tilt.git
git clone https://github.com/phopley/rodney_missions.git
git clone https://github.com/lrse/ros-keyboard.git
git clone https://github.com/phopley/servo_msgs.git
git clone https://github.com/phopley/speech.git
git clone https://github.com/phopley/thunderborg.git
git clone https://github.com/phopley/tacho_msgs.git
git clone https://github.com/Slamtec/rplidar_ros.git

