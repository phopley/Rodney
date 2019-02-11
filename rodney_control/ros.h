#ifndef _ROS_H_
#define _ROS_H_

#include <ArduinoHardware.h>
#include <ros/node_handle.h>


namespace ros
{
  // default 25, 25, 512, 512
  typedef NodeHandle_<ArduinoHardware, 25, 25, 512, 1024> NodeHandle;    
}

#endif
