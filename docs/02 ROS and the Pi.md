# ROS and the Raspberry Pi
So I have already made the decision to use a Raspberry Pi 3, Model B with 1GB of RAM as the main processor and to use the [Robot Operating System](http://wiki.ros.org/ "ROS"). So the first stage is to install the ROS on a Pi.

Now I could have used a Raspberry Pi Raspbain image and installed ROS onto that image, but I have come across [Ubiquity Robotics](https://ubiquityrobotics.com "Ubiquity") who make robots but also make their Raspberry Pi image which includes ROS available for free.

Raspberry Pi 3 images for Ubuntu Linux with ROS are available for [download](https://downloads.ubiquityrobotics.com/pi.html "Image"). The great thing about this image it includes some of their ROS nodes already built in like the node for the Pi Camera, [raspicam_node](https://github.com/ubiquityRobotics/raspicam_node "raspicam_node"). Even if you go with another Linux image and install ROS on your own you can still make use of nodes from Ubiquity by downloading the code from [their GitHub site](https://github.com/UbiquityRobotics "Ubiquity").
