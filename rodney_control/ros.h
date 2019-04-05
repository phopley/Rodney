/* 
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote prducts derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
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
