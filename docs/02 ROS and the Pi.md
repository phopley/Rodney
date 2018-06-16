# ROS and the Raspberry Pi
So I have already made the decision to use a Raspberry Pi 3, Model B with 1GB of RAM as the main processor and to use the [Robot Operating System](http://wiki.ros.org/ "ROS"). So the first stage is to install the ROS on a Pi.

Now I could have used a Raspberry Pi Raspbain image and installed ROS onto that image, but I have come across [Ubiquity Robotics](https://ubiquityrobotics.com "Ubiquity") who make robots but also make their Raspberry Pi image which includes ROS available for free.

Raspberry Pi 3 images for Ubuntu Linux with ROS are available for [download](https://downloads.ubiquityrobotics.com/pi.html "Image"). The ROS version included in this image is the Kinetic version. The great thing about this image it includes some of their ROS nodes already built in like the node for the Pi Camera, [raspicam_node](https://github.com/ubiquityRobotics/raspicam_node "raspicam_node"). Even if you go with another Linux image and install ROS on your own you can still make use of nodes from Ubiquity by downloading the code from [their GitHub site](https://github.com/UbiquityRobotics "Ubiquity").

Other Raspberry Pi peripherals that I intend to use on the Rodney project are:
- 7" Touchscreen Display
- Camera Module V2

The plan is to use the screen for passing status information to a user, web content and also for displaying an animated robot face. The camera will be the eyes of the robot initially being used for facial recognition.

The following images show the 7" display with the Raspberry Pi and camera mounted on the rear of the screen. The camera is mounted using a 3D printed bracket. The stl file for the bracket is available [here]( https://github.com/phopley/rodney/blob/master/docs/3D%20Prints/camera%20bracketV2.stl "Camera bracket")

<img src="https://github.com/phopley/rodney/blob/master/docs/images/IMG_0380.JPG" width="427" height="284" title="7inch screen"> <img src="https://github.com/phopley/rodney/blob/master/docs/images/IMG_0381.JPG" width="427" height="284" title="7inch screen and camera">

The ROS system will run across a distributed network, I have therefore also installed ROS on a Ubuntu desktop I have available. This PC will be used to develop the nodes for the system and to run some of the ROS tools available to test the system.
