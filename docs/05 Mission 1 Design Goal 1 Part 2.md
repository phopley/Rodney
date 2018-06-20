## Face Recognition
The pan_tilt node was written in C++, this node will be written in Python. This partly because it gives us examples of ROS nodes written in two different languages but also because I'm going to write the node around some existing face recognition Python code.

The package is available in the GitHub Repository https://github.com/phopley/face_recognition. See the package documentation for details. This package not only contains the code for the ROS node but also two standalone Python scripts that are used to train system for the faces you wish to recognise.

The `data_set_generator.py` script is used to capture images of each subject. It uses the Raspberry Pi Camera to capture facial images and stores them ready for the face recognition training phase. The script should be run for each subject that you wish to recognise. Ensure when prompted you give the next unique ID value (start at 1) and increase for each subject recorded. As well as a unique ID the script prompts for the subject name and wether  it is low light conditions. Ideally the script should be run twice per subject (giving the same ID) in good light and low light conditions.

Once the `data_set_generator.py` script has been run the next stage is to run the `training.py` script, this script examines and the images in the data set and produces a training file which is load by the ROS node. Each time a new subject is added to the data set, using the `data_set_generator.py` script, the `training.py` script should be re-run.

The face_recognition node itself, when instructed examines the next frame from the Pi camera and attempts to detect a face(s). If a face(s) is detected it them attempts to identify the face(s) from those that it has been trained with. A ROS message is sent from the node containing the unique ID and names of any faces identified. If no faces are identified the data in the message will be empty. As well as this message is also publishes a messages which contains a copy of the modified image. If a face(s) was detected this modified image includes a square drawn around each face along with text showing the name of the subject and a confidence level indicating  how confident the software is that the name is correct. The package includes a confidence_level configurable parameter which is used to set a threshold level used to accept the identification. If the confidence level for a detection is below this threshold then the square on the modified image is red in colour and the message with ID and name will be empty. If the confidence level for a detection is above this threshold then the square on the modified image is green in colour and the name and ID will appear in the other message.

The message containing the ID and name is once again a user defined message. It is available in the ROS package face_recognition_msgs. This package also contains an action which is used to trigger a complete scan, which includes moving the head with the pan/tilt device. This package is available in the GitHub Repository https://github.com/phopley/face_recognition_msgs. See the package documentation for full details.

To obtain images from the Raspberry Pi camera I'm going to use the Ubiquity Robotics ROS node, raspicam. If using the Ubiquity Raspberry Pi image then this package is available in the ROS build, if using a different OS image then it's available from the GitHub Repository https://github.com/UbiquityRobotics/raspicam_node.

ROS passes images using its own message type, however to manipulate the image the face recognition code converts it to an OpenCV image using a ROS package called cv_bridge. The image I'm going to use from the camera is compressed, but although the C++ version of cv_bridge can handle this compressed image, the Python version currently cannot. To get around this anomaly I'm going to use another available ROS node of type republish, which is part of the image_transport package. This node will convert the compressed image into a raw image and it's this raw image message that the face_recognition node subscribes to.

The face_recognition package also contains a launch file to test the camera and the the face recognition node. The launch file will launch the camera node, the republish node and the face_recognition node.
``` XML
<?xml version="1.0" ?>
<launch>
  <rosparam command="load" file="$(find face_recognition)/config/config.yaml" /> 
  <include file="$(find raspicam_node)/launch/camerav2_1280x960.launch" />
  <node name="republish" type="republish" pkg="image_transport" output="screen" args="compressed in:=/raspicam_node/image/ raw out:=/camera/image/raw" />
  <node pkg="face_recognition" type="face_recognition_node.py" name="face_recognition_node" />
</launch>
```
To run the nodes using the launch file, `cd` to the workspace where the face_recognition package was built, source the setup bash file and call roslaunch.
```
cd ~/catkin_ws
source devel/setup.bash
roslaunch face_recognition test.launch
```
The following ROS Graph depicts the face recognition part of the system.

![alt text](https://github.com/phopley/rodney/blob/master/docs/images/rosgraph_face_rec.png "face recognition graph")

With the system running as shown in the graph above do the following:
- Start an instance of `rqt_image_view` and select the `/face_recognition_node/adjusted_image` topic for viewing
- Set up a teminal window to echo any results `rostopic echo /face_recognition_node/result`
- In another terminal trigger a face recognition scan with using rostopic and the following command `rostopic pub -1 /face_recognition_node/start std_msgs/Empty`

As the scan was started I was looking at the camera and obtained the following results. The terminal running the rostopic echo reported
```![alt text](
ids_detected:[1]
names_detected: [Phil]
---
```
The rqt_image_view application showed the following image:
![alt text](https://github.com/phopley/rodney/blob/master/docs/images/image_face_rec.png "face recognition image")

Note that during this test raspicam_node, republish and face_recognition_node were all running on the Raspberry Pi but the debug tools, rostopic and rqt_image_view were running on a PC running Ubuntu. For a tutorial on running a distributed system see the [following tutorial](http://wiki.ros.org/ROS/Tutorials/MultipleMachines "Multiple Machines") 
