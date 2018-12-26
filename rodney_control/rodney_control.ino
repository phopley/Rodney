/*
 * This version controls upto four RC ServosnoInterruptCount and publishes the tacho message monitoring
 * two wheels.
 * 
 * The node subscribes to the servo topic and acts on a rodney_msgs::servo_array message.
 * This message contains two elements, index and angle. Index references the servos 0-3 and 
 * angle is the angle to set the servo to 0-180.
 *
 *
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

/* Define the PWM pins that the servos are connected to */
#define SERVO_0 9
#define SERVO_1 6
#define SERVO_2 5
#define SERVO_3 10

#define MISS_INT_COUNT 3
#define ENCOUNDER_COUNTS_FULL_REV 12

ros::NodeHandle  nh;

Servo servo0;
Servo servo1;
Servo servo2;
Servo servo3;

tacho_msgs::tacho tachoMsg;

volatile bool interrupt0 = false;
volatile bool interrupt1 = false;
volatile unsigned long time0 = 0;
volatile unsigned long time1 = 0;
volatile unsigned long timeLast0 = 0;
volatile unsigned long timeLast1 = 0;
volatile unsigned int noInterruptCount0 = 0;
volatile unsigned int noInterruptCount1 = 0;

void servo_cb( const servo_msgs::servo_array& cmd_msg){
  
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
    
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}

ros::Subscriber<servo_msgs::servo_array> sub("servo", servo_cb);
ros::Publisher pub("tacho", &tachoMsg);

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
    
  servo0.attach(SERVO_0); //attach it to the pin
  servo1.attach(SERVO_1);
  servo2.attach(SERVO_2);
  servo3.attach(SERVO_3);

  // Attach the interrupts for the tachos
  attachInterrupt(0, int0, RISING); // Int0 is pin 2
  attachInterrupt(1, int1, RISING); // Int1 is pin 3
  
  // Defaults
  servo0.write(90);
  servo1.write(120);
}

long publisherTime;

void loop(){
  // Is it time to publish the tacho message
  if(millis() > publisherTime) {
    if(interrupt0 == false)
    {
      noInterruptCount0++;

      if(noInterruptCount0 >= MISS_INT_COUNT)
      {
        // No new interrupt for a while so reset the rpm to zero
        noInterruptCount0 = MISS_INT_COUNT;
        tachoMsg.rwheelrpm = 0.0;          
      } // Otherwise just use the rpm set previously
    }
    else
    {
      tachoMsg.rwheelrpm = (1000000.0/((float)(time0*ENCOUNDER_COUNTS_FULL_REV)))*60;
      
      interrupt0 = false;      
    }

    if(interrupt1 == false)
    {
      noInterruptCount1++;   

      if(noInterruptCount1 >= MISS_INT_COUNT)
      {
        // No new interrupt for a while so reset the velocity to zero
        noInterruptCount1 = MISS_INT_COUNT;
        tachoMsg.lwheelrpm = 0.0;      
      }// Otherwise just use the rpm set previously
    }
    else
    {
      tachoMsg.lwheelrpm = (1000000.0/((float)(time1*ENCOUNDER_COUNTS_FULL_REV)))*60;
      interrupt1 = false;
    }    

    pub.publish(&tachoMsg);

    publisherTime = millis() + 50; // Publish at 20Hz
  }
  
  nh.spinOnce();
}

/*  Each interrupt keeps track of the time between interrupts.
 *  Our max speed is around 1m/s and the encoder gives an interrupt 12 times per revolution.
 *  With a wheel circumference of 0.34m, that means the minimum time between interrupts is 
 *  approx 28ms, so lets ignore enything less than 20ms and assume its spurious.
 *  The IR is affected by sunlight reflections so this is an attempt to help filter out 
 *  spurious detections.
 */
void int0 () {
  if((micros() - timeLast0) > 20000)
  {
    if(digitalRead(2) == 1)
    {
      time0 = micros() - timeLast0;
      timeLast0 = micros();
      interrupt0 = true;
      noInterruptCount0 = 0;
    }
  }
  else
  {
    // Spurious detection. Reset timer, interrupts will synch back up
    timeLast0 = micros();
    noInterruptCount0 = 0;    
  }
}

void int1 () {
  if((micros() - timeLast1) > 20000)
  {
    if(digitalRead(3) == 1)
    {
      time1 = micros() - timeLast1;
      timeLast1 = micros();
      interrupt1 = true;
      noInterruptCount1 = 0;
    }
  }
  else
  {
    // Spurious detection. Reset timer, interrupts will synch back up
    timeLast1 = micros();
    noInterruptCount1 = 0;
  }
}
