/*
 * This version: 
 * - Controls upto four RC Servos on the servo topic
 * - Publishes the tacho on the tacho topic monitoring two motors with Hall sensors.
 * 
 * The node subscribes to the servo topic and acts on a rodney_msgs::servo_array message.
 * This message contains two elements, index and angle. Index references the servos 0-3 and 
 * angle is the angle to set the servo to 0-180.
 * 
 *
 * The connections to a Teensy 3.5 are:  
 * Pin 0 (INT)   -> used for monitoring right motor speed
 * Pin 1 (Input) -> used for sensing right motor direction
 * Pin 3 (INT)   -> used for monitoring left motor speed
 * Pin 4 (Input) -> used for sensing left motor direction 
 * Pin 20 (PWM)  -> servo indexed 3
 * Pin 21 (PWM)  -> servo indexed 2 
 * Pin 22 (PWM)  -> servo indexed 1
 * PIN 23 (PWM)  -> servo indexed 0 
 * 
 * The connections to a Nano are:
 * D2 (INT)   -> used for monitoring right motor speed
 * D4 (Input) -> used for sensing right motor direction
 * D3 (INT)   -> used for monitoring left motor speed
 * D7 (Input) -> used for sensing left motor direction 
 * D10 (PWM)  -> servo indexed 3
 * D5 (PWM)  -> servo indexed 2 
 * D6 (PWM)  -> servo indexed 1
 * D9 (PWM)  -> servo indexed 0  
 * 
 */
#if defined(__MK64FX512__)
#include <PWMServo.h> // Use PWMServo if using a Teensy
#include "ros.h"      // Use "ros.h" not <ros.h> so that by using our local version we can increase buffer size if required
#else
#include <Servo.h>
#include <ros.h>
#endif
#include <servo_msgs/servo_array.h>
#include <tacho_msgs/tacho.h>

void servo_cb( const servo_msgs::servo_array& cmd_msg);
void WheelSpeed0();
void WheelSpeed1();

#define LED_PIN 13  // Onboard LED

#define GEAR_BOX_COUNTS_PER_REV 1440.0f

#if defined(__MK64FX512__) 
// Teensy
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

PWMServo  servo0;
PWMServo  servo1;
PWMServo  servo2;
PWMServo  servo3;
#else
// Define the PWM pins that the other servos are connected to
#define SERVO_0 9
#define SERVO_1 9
#define SERVO_2 5
#define SERVO_3 10

// Define pins used for two Hall sensors
#define ENCODER0_PINA 2  // Interrupt 0
#define ENCODER0_PINB 4  // Digital pin
#define ENCODER1_PINA 3  // Interrupt 1
#define ENCODER1_PINB 7  // Digital pin

Servo  servo0;
Servo  servo1;
Servo  servo2;
Servo  servo3;
#endif

tacho_msgs::tacho      tachoMsg;

ros::NodeHandle nh;

ros::Publisher tachoPub("tacho", &tachoMsg);
ros::Subscriber<servo_msgs::servo_array> subServo("servo", servo_cb);

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
  nh.advertise(tachoPub);
  nh.subscribe(subServo);

  // Attach servos
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

  // Turn on the onboard LED to show we are running 
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
}


void loop()
{
  // Is it time to publish the tacho message
  if(millis() > publisherTime)
  {
    float deltaTime;
    
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
