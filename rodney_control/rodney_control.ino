/*
 * This version: 
 * - Controls upto four RC Servos on the servo topic
 * - Publishes the tacho on the tacho topic monitoring two motors with Hall sensors.
 * - Controls a servo attached to a LIDAR and takes range measurments from the LIDAR publishing them on the laserScan topic
 * 
 * The node subscribes to the servo topic and acts on a rodney_msgs::servo_array message.
 * This message contains two elements, index and angle. Index references the servos 0-3 and 
 * angle is the angle to set the servo to 0-180.
 * 
 * The node also subscribes to the /commands/lidar_enable topic the acts on a std_msgs::Bool message.
 * If the data in the message is true the use of the LIDAR is enabled, otherwise it is disabled. 
 *
 * The connections to a Teensy 3.5 are:  
 * Pin 0 (INT)   -> used for monitoring right motor speed
 * Pin 1 (Input)   -> used for sensing right motor direction
 * Pin 2 (PWM)   -> LIDAR Servo control signal 
 * Pin 3 (INT)   -> used for monitoring left motor speed
 * Pin 4 (Input) -> used for sensing left motor direction 
 * Pin 18 (SCL)  -> LIDAR SCL (Green wire on Garmin LIDAR-Lite V3)
 * Pin 19 (SDA)  -> LIDAR SDA (Blue wire on Garmin LIDAR-Lite V3)
 * Pin 20 (PWM)  -> servo indexed 3
 * Pin 21 (PWM)  -> servo indexed 2 
 * Pin 22 (PWM)  -> servo indexed 1
 * PIN 23 (PWM)  -> servo indexed 0
 * 
 */

#include <PWMServo.h>
#include <LIDARLite.h>
#include "ros.h"  // USE "ros.h" not <ros.h> SO THAT WE USE OUR LOCAL VERSION
#include <sensor_msgs/LaserScan.h>
#include <servo_msgs/servo_array.h>
#include <tacho_msgs/tacho.h>
#include <std_msgs/Bool.h>

void servo_cb( const servo_msgs::servo_array& cmd_msg);
void command_lidar_cb( const std_msgs::Bool& cmd_msg);
void WheelSpeed0();
void WheelSpeed1();

#define SERVO_LIDAR 2 // LIDAR Servo

// Define the PWM pins that the other servos are connected to
#define SERVO_0 23
#define SERVO_1 22
#define SERVO_2 21
#define SERVO_3 20

// Define pins used for two Hall sensors
#define ENCODER0_PINA 0  // Interrupt
#define ENCODER0_PINB 1  // Digital pin
#define ENCODER1_PINA 3  // Interrupt
#define ENCODER1_PINB 4  // Digital pin

#define LED_PIN 13  // Onboard LED

#define GEAR_BOX_COUNTS_PER_REV 1440.0f

#define NUM_READINGS 180 // 1 each degree
#define TRUE_MIN_ANGLE (-1.571)
#define TRUE_MAX_ANGLE (1.571)
#define ANGLE_INCREMENT (3.142 / NUM_READINGS)

PWMServo  servoLidar;
PWMServo  servo0;
PWMServo  servo1;
PWMServo  servo2;
PWMServo  servo3;
LIDARLite myLidarLite;

tacho_msgs::tacho      tachoMsg;
sensor_msgs::LaserScan lidarMsg;

ros::NodeHandle nh;

ros::Publisher lidarPub("/laser_scan", &lidarMsg);
ros::Publisher tachoPub("tacho", &tachoMsg);
ros::Subscriber<servo_msgs::servo_array> subServo("servo", servo_cb);
ros::Subscriber<std_msgs::Bool> subLidarCmd("/commands/lidar_enable", command_lidar_cb);

float deltaTime;
float ranges[NUM_READINGS];
bool  laserOn;        // True when the LIDAR is enabled
bool  incDir;         // True when direction of LIDAR servo is increasing angle
bool  newScan;        // True when the start of a new scanning sequence is due
int   scanIndex;      // Current scan position gives servo position and array position for ranges
byte  encoder0PinALast;
byte  encoder1PinALast;
volatile int encoder0Count; // Number of pulses
volatile int encoder1Count; // Number of pulses
volatile boolean encoder0Direction; //Rotation direction
volatile boolean encoder1Direction; //Rotation direction
unsigned long publisherTime;
unsigned long currentTime;
unsigned long lastTime;


void setup() 
{
  nh.initNode();
  nh.advertise(lidarPub);
  nh.advertise(tachoPub);
  nh.subscribe(subServo);
  nh.subscribe(subLidarCmd);

  // Attach LIDAR  servo. The servo I'm using goes back past 0 degrees
  // so setting the min and maximum (defaults are 544, 2400)
  servoLidar.attach(SERVO_LIDAR, 620, 2400);
  servoLidar.write(0); // Start at 0 degrees (-1.571  radians for ROS)

  // Attach other servos
  servo0.attach(SERVO_0); //attach it to the pin
  servo1.attach(SERVO_1);
  servo2.attach(SERVO_2);
  servo3.attach(SERVO_3);
  servo0.write(90);
  servo1.write(120);
  servo2.write(90);
  servo3.write(90);

  encoder0Direction = true;   // default is forward
  encoder1Direction = true;
  encoder0Count = 0;
  encoder1Count = 0;

  pinMode(ENCODER0_PINB, INPUT);
  pinMode(ENCODER1_PINB, INPUT);

  // Attach the interrupts for the Hall sensors
  attachInterrupt(digitalPinToInterrupt(ENCODER0_PINA), WheelSpeed0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER1_PINA), WheelSpeed1, CHANGE);
    
  myLidarLite.begin(0, true);
  myLidarLite.configure(0);

  // To determine the end of the array each one has an auto-generated 
  // integer companion with the same name and the suffix _length
  lidarMsg.ranges_length = NUM_READINGS;
  lidarMsg.intensities_length = 0;
  lidarMsg.header.frame_id = "lidar_link";
  lidarMsg.range_min = 0.0;
  lidarMsg.range_max = 40.0;
  // These will change sign when the servo moves in the opposite direction
  lidarMsg.angle_min = TRUE_MIN_ANGLE;
  lidarMsg.angle_max = TRUE_MAX_ANGLE;
  lidarMsg.angle_increment = ANGLE_INCREMENT;

  scanIndex = 0;
  incDir = true;
  newScan = true;
  laserOn = false;

  // Turn on the onboard LED to show we are running 
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
}


void loop()
{
  if(laserOn == true)
  {
    // Move the LIDAR servo
    servoLidar.write(scanIndex);
  }

  // Is it time to publish the tacho message
  if(millis() > publisherTime)
  {
    currentTime = micros();
    deltaTime = (float)(currentTime - lastTime)/1000000.0;

    // Right wheel speed
    tachoMsg.rwheelrpm = (((((float)encoder0Count)/2.0f)/deltaTime)/GEAR_BOX_COUNTS_PER_REV)*60.0f;
    encoder0Count = 0;

    // Left wheel speed
    tachoMsg.lwheelrpm = (((((float)encoder1Count)/2.0f)/deltaTime)/GEAR_BOX_COUNTS_PER_REV)*60.0f;
    encoder1Count = 0;

    lastTime = currentTime;
    
    tachoPub.publish(&tachoMsg);
    publisherTime = millis() + 50; // Publish at 20Hz
  }

  // Is the LIDAR enabled
  if(laserOn == true)
  {
    if(incDir == true)
    {
      if(newScan == true)
      {
        // Moving anti-clockwise around Z, angle increasing with ROS standard
        lidarMsg.angle_min = TRUE_MIN_ANGLE;
        lidarMsg.angle_max = TRUE_MAX_ANGLE;
        lidarMsg.angle_increment = ANGLE_INCREMENT;

        // Time is acquisition time of first ray in scan
        lidarMsg.header.stamp = nh.now();
  
        newScan = false;
      }

      // Read the range from the LIDAR
      // Receiver bias correction must be performed periodically e.g. 1 out of every 100
      if((scanIndex%100) == 0)
      {
        ranges[scanIndex] = (float)myLidarLite.distance() / 100.0f;
      }
      else
      {
        ranges[scanIndex] = (float)myLidarLite.distance(false) / 100.0f;
      }

      scanIndex++;

      if(scanIndex == NUM_READINGS)
      {
        // End of the scan sequence
        lidarMsg.ranges = ranges;  
        lidarPub.publish(&lidarMsg);

        // Setup to move in the other direction
        scanIndex = NUM_READINGS-1;
        newScan = true;
        incDir = false;
      }
    }
    else
    {
      if(newScan == true)
      {
        // Moving clockwise around Z, angle decreasing with ROS standard
        lidarMsg.angle_min = -TRUE_MIN_ANGLE;
        lidarMsg.angle_max = -TRUE_MAX_ANGLE;
        lidarMsg.angle_increment = -ANGLE_INCREMENT;

        // Time is acquisition time of first ray in scan
        lidarMsg.header.stamp = nh.now();
  
        newScan = false;
      } 

      // Read the range from the LIDAR
      // Receiver bias correction must be performed periodically e.g. 1 out of every 100
      if((scanIndex%100) == 0)
      {
        ranges[scanIndex] = (float)myLidarLite.distance() / 100.0f;
      }
      else
      {
        ranges[scanIndex] = (float)myLidarLite.distance(false) / 100.0f;
      }

      scanIndex--;

      if(scanIndex < 0)
      {
        // End of the scan sequence
        lidarMsg.ranges = ranges;  
        lidarPub.publish(&lidarMsg);

        // Setup to move in the other direction
        scanIndex = 0;
        newScan = true;
        incDir = true;
      }      
    }    
  }
  
  nh.spinOnce();
}


// Callback for when servo array message received
void servo_cb( const servo_msgs::servo_array& cmd_msg)
{
  /* Which servo to drive */
  switch(cmd_msg.index)
  {
    case 0:      
      servo0.write(cmd_msg.angle); //set servo 0 angle, should be from 0-180
      break;

    case 1:
      servo1.write(cmd_msg.angle); //set servo 1 angle, should be from 0-180
      break;

    case 2:
      servo2.write(cmd_msg.angle); //set servo 2 angle, should be from 0-180
      break;

    case 3:
      servo3.write(cmd_msg.angle); //set servo 3 angle, should be from 0-180
      break;
      
    default:
      nh.logdebug("Error incorrect servo index");
      break;
  }
}


// Callback for when lidar enable command message is received
void command_lidar_cb( const std_msgs::Bool& cmd_msg)
{
  // Turn the LIDAR scan on or off
  laserOn = cmd_msg.data;

  if(laserOn == true)
  {
    // Starting the scan from RC 0 degrees and increasing
    scanIndex = 0;
    newScan = true;
    incDir = true;
  }
  else
  {
    // Set servo to start of scan for when next enabled
    servoLidar.write(0);
  }    
}


// ISR
void WheelSpeed0()
{
  int state = digitalRead(ENCODER0_PINA);

  if((encoder0PinALast == LOW) && (state == HIGH))
  {
    int val = digitalRead(ENCODER0_PINB);

    if(val == LOW && encoder0Direction)
    {
      encoder0Direction = false; // Reverse
    }
    else if (val == HIGH && !encoder0Direction)
    {
      encoder0Direction = true; // Forward
    }
  }

  encoder0PinALast = state;

  if(!encoder0Direction)
  {
    encoder0Count++;
  }
  else
  {
    encoder0Count--;
  }
}


// ISR
void WheelSpeed1()
{
  int state = digitalRead(ENCODER1_PINA);

  if((encoder1PinALast == LOW) && (state == HIGH))
  {
    int val = digitalRead(ENCODER1_PINB);

    if(val == LOW && encoder1Direction)
    {
      encoder1Direction = false; // Reverse
    }
    else if (val == HIGH && !encoder1Direction)
    {
      encoder1Direction = true; // Forward
    }
  }

  encoder1PinALast = state;

  if(!encoder1Direction)
  {
    encoder1Count++;
  }
  else
  {
    encoder1Count--;
  }
}
