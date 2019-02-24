#ifndef _ROS_H_
#define _ROS_H_

// As we are no longer including any .h file in the root of the 
// ros_lib library the IDE can't find ros/node_handle.h
// Add the dummy.h empty file in the ros_lib root
#include <dummy.h>
#include <ros/node_handle.h>
#include "ArduinoHardware.h"


namespace ros
{
  
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
  // Teensy 3.5 or 6.3
  typedef NodeHandle_<ArduinoHardware, 25, 25, 512, 512> NodeHandle;
#elif defined(__AVR_ATmega328P__)
  // arduino Nano
  // 10 publishers, 15 subscribers, 128 bytes input buffer and 256 bytes output buffer
  typedef NodeHandle_<ArduinoHardware, 10, 15, 128, 256> NodeHandle;
#else
  typedef NodeHandle_<ArduinoHardware> NodeHandle; // default 25, 25, 512, 512 
#endif 
}

#endif
