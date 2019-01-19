/*
 * This version controls upto four RC Servos and publishes the tacho message monitoring
 * two motors.
 * 
 * The node subscribes to the servo topic and acts on a rodney_msgs::servo_array message.
 * This message contains two elements, index and angle. Index references the servos 0-3 and 
 * angle is the angle to set the servo to 0-180.
 *
 * D2 -> INT0 used for monitoring right motor speed
 * D3 -> INT1 used for monitoring left motor speed
 * D4 -> Digital input used for sensing right motor direction
 * D5 -> PWM servo indexed 2
 * D6 -> PWM servo indexed 1
 * D7 -> Digital input used for sensing left motor direction
 * D9 -> PWM servo indexed 0
 * D10 -> PWM servo indexed 3
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <servo_msgs/servo_array.h>
#include <tacho_msgs/tacho.h>

// Define the PWM pins that the servos are connected to
#define SERVO_0 9
#define SERVO_1 6
#define SERVO_2 5
#define SERVO_3 10

// Define pins used for two Hall sensors
#define ENCODER0_PINA 2 // Interrupt 0
#define ENCODER0_PINB 4
#define ENCODER1_PINA 3 // Interrupt 1
#define ENCODER1_PINB 7

#define GEAR_BOX_COUNTS_PER_REV 1440.0f

ros::NodeHandle  nh;

Servo servo0;
Servo servo1;
Servo servo2;
Servo servo3;

tacho_msgs::tacho tachoMsg;

byte encoder0PinALast;
byte encoder1PinALast;
volatile int encoder0Count; // Number of pulses
volatile int encoder1Count; // Number of pulses
volatile boolean encoder0Direction; //Rotation direction
volatile boolean encoder1Direction; //Rotation direction

void servo_cb( const servo_msgs::servo_array& cmd_msg)
{  
  /* Which servo to drive */
  switch(cmd_msg.index)
  {
    case 0:
      nh.logdebug("Servo 0 ");
      servo0.write(cmd_msg.angle); //set servo 0 angle, should be from 0-180
      break;

    case 1:
      nh.logdebug("Servo 1 ");
      servo1.write(cmd_msg.angle); //set servo 1 angle, should be from 0-180
      break;

    case 2:
      nh.logdebug("Servo 2 ");
      servo2.write(cmd_msg.angle); //set servo 2 angle, should be from 0-180
      break;

    case 3:
      nh.logdebug("Servo 3 ");
      servo3.write(cmd_msg.angle); //set servo 3 angle, should be from 0-180
      break;
      
    default:
      nh.logdebug("No Servo");
      break;
  }
}

ros::Subscriber<servo_msgs::servo_array> sub("servo", servo_cb);
ros::Publisher pub("tacho", &tachoMsg);

void setup()
{
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
    
  servo0.attach(SERVO_0); //attach it to the pin
  servo1.attach(SERVO_1);
  servo2.attach(SERVO_2);
  servo3.attach(SERVO_3);

  encoder0Direction = true;   // default is forward
  encoder1Direction = true;
  encoder0Count = 0;
  encoder1Count = 0;

  pinMode(ENCODER0_PINB, INPUT);
  pinMode(ENCODER1_PINB, INPUT);
  
  // Attach the interrupts for the Hall sensors
  attachInterrupt(0, WheelSpeed0, CHANGE); // Int0 is pin 2
  attachInterrupt(1, WheelSpeed1, CHANGE); // Int1 is pin 3
  
  // Defaults
  servo0.write(90);
  servo1.write(120);
}

unsigned long publisherTime;
unsigned long currentTime;
unsigned long lastTime;
float deltaTime;

void loop()
{
  // Is it time to publish the tacho message
  if(millis() > publisherTime)
  {
    currentTime = micros();
    deltaTime = (float)(currentTime - lastTime)/1000000.0;

    // Right wheel speed
    tachoMsg.rwheelrpm = ((((float)encoder0Count)/deltaTime)/GEAR_BOX_COUNTS_PER_REV)*60.0f;
    encoder0Count = 0;

    // Left wheel speed
    tachoMsg.lwheelrpm = ((((float)encoder1Count)/deltaTime)/GEAR_BOX_COUNTS_PER_REV)*60.0f;
    encoder1Count = 0;

    lastTime = currentTime;
    
    pub.publish(&tachoMsg);
    publisherTime = millis() + 50; // Publish at 20Hz
  }
  
  nh.spinOnce();
}

// ISR 0
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

// ISR 1
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
