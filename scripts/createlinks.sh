#!/bin/bash
# script to create symbolic links from git folders to rodney_ws for Rodney Robot project
# run ./createlinks
ln -s ~/git/face_recognition ~/rodney_ws/src
ln -s ~/git/face_recognition_msgs ~/rodney_ws/src
ln -s ~/git/head_control ~/rodney_ws/src
ln -s ~/git/imu_calib ~/rodney_ws/src
ln -s ~/git/pan_tilt ~/rodney_ws/src
ln -s ~/git/pi_io.git ~/rodney_ws/src
ln -s ~/git/rodney ~/rodney_ws/src
ln -s ~/git/rodney_missions ~/rodney_ws/src
ln -s ~/git/ros-keyboard ~/rodney_ws/src
ln -s ~/git/rplidar_ros ~/rodney_ws/src
ln -s ~/git/servo_msgs ~/rodney_ws/src
ln -s ~/git/speech ~/rodney_ws/src
ln -s ~/git/tacho_msgs ~/rodney_ws/src
ln -s ~/git/thunderborg ~/rodney_ws/src


