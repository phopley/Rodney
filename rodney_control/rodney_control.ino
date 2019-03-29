/*
 * This version: 
 * - Controls upto four RC Servos on the servo topic
 * - Publishes the tacho on the tacho topic monitoring two motors with Hall sensors.
 * - Publish imu data on the imu/data_raw topic
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
 * Pin 18 (SDA0) -> MPU9250 SDA
 * Pin 19 (SCL0) -> MPU9250 SCL
 * Pin 20 (PWM)  -> servo indexed 3
 * Pin 21 (PWM)  -> servo indexed 2 
 * Pin 22 (PWM)  -> servo indexed 1
 * Pin 23 (PWM)  -> servo indexed 0 
 */
#include <PWMServo.h> // Use PWMServo on Teensy
#include <MPU9250.h>

// Use "ros.h" not <ros.h> so that by using our local version 
// we can increase/decrease buffer size if required and 
// increased the baud rate on faster boards.
#include "ros.h"
#include <servo_msgs/servo_array.h>
#include <tacho_msgs/tacho.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

void servo_cb( const servo_msgs::servo_array& cmd_msg);
void WheelSpeed0();
void WheelSpeed1();

#define LED_PIN 13  // Onboard LED

#define GEAR_BOX_COUNTS_PER_REV 1440.0f

// Define the period in milliseconds between tacho messages
#define TACHO_PERIOD_MS 50  // Publish at 20Hz

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

#define G_TO_MS2 9.80665

#define I2Cclock 400000
#define I2Cport Wire
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0
MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);

tacho_msgs::tacho tachoMsg;
sensor_msgs::Imu  imuMsg;  

ros::NodeHandle nh;

ros::Publisher tachoPub("tacho", &tachoMsg);
ros::Publisher imuPub("imu/data_raw", &imuMsg);
ros::Subscriber<servo_msgs::servo_array> subServo("servo", servo_cb);

bool  imuSelfTestPass;
byte  encoder0PinALast;
byte  encoder1PinALast;
volatile int encoder0Count; // Number of pulses
volatile int encoder1Count; // Number of pulses
volatile boolean encoder0Direction; //Rotation direction
volatile boolean encoder1Direction; //Rotation direction
unsigned long publisherTime;
unsigned long currentTime;
unsigned long lastTime;
char imu_link[] = "imu";


void setup() 
{
  Wire.begin();
  
  nh.initNode();
  nh.advertise(tachoPub);
  nh.advertise(imuPub);
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

  // Read the WHO_AM_I register of the IMU, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  
  if(c == 0x71) // WHO_AM_I should always be 0x71
  {
    // Start by performing self test
    myIMU.MPU9250SelfTest(myIMU.selfTest);

    imuSelfTestPass = true;
    
    for(int i = 0; i < 6; i++)
    {
      if(abs(myIMU.selfTest[i]) > 14.0f)
      {
        imuSelfTestPass = false;
      }
    }

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    // Initialize device for active mode read of acclerometer, gyroscope, and temperature
    myIMU.initMPU9250();

    // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);

    if(d == 0x48)
    {
      // Get magnetometer calibration from AK8963 ROM
      // Initialize device for active mode read of magnetometer
      myIMU.initAK8963(myIMU.factoryMagCalibration);

      // Get sensor resolutions, only need to do this once
      myIMU.getAres();
      myIMU.getGres();
      myIMU.getMres();      
    }
    else
    {
      imuSelfTestPass = false;
    }
  }
  else
  {
    imuSelfTestPass = false;
  }

  if(imuSelfTestPass == true)
  {
    // Turn on the onboard LED to show we are running 
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
  }
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
    publisherTime = millis() + TACHO_PERIOD_MS;
  }

  // IMU 
  if(imuSelfTestPass == true)
  {
    // Check to see if all data registers have new data
    if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
    {
      myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values

      // Now we'll calculate the accleration value into actual g's
      // This depends on scale being set
      myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes;
      myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes;
      myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes;

      myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values

      // Calculate the gyro value into actual degrees per second
      // This depends on scale being set
      myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
      myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
      myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

      myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values

      // Calculate the magnetometer values in milliGauss
      // Include factory calibration per data sheet and user environmental corrections
      // Get actual magnetometer value, this depends on scale being set
      myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes
                 * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
      myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes
                 * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
      myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes
                 * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
    }
                               
    // Is it time to publish IMU data
    myIMU.delt_t = millis() - myIMU.count;
    if (myIMU.delt_t > 50)
    {
      // IMU
      imuMsg.header.frame_id = imu_link;
      imuMsg.header.stamp = nh.now();

      // We are not providing orientation so the 
      // first element of the this matrix should be -1
      imuMsg.orientation_covariance[0] = -1;

      imuMsg.angular_velocity.x = myIMU.gx * DEG_TO_RAD;
      imuMsg.angular_velocity.y = myIMU.gy * DEG_TO_RAD;
      imuMsg.angular_velocity.z = myIMU.gz * DEG_TO_RAD;

      // angular velocity covariance
      imuMsg.angular_velocity_covariance[0] = 0.00002;
      imuMsg.angular_velocity_covariance[4] = 0.00002;
      imuMsg.angular_velocity_covariance[8] = 0.00002;

      imuMsg.linear_acceleration.x = myIMU.ax * G_TO_MS2;
      imuMsg.linear_acceleration.y = myIMU.ay * G_TO_MS2;
      imuMsg.linear_acceleration.z = myIMU.az * G_TO_MS2;
      
      // linear acceleration covariance
      imuMsg.linear_acceleration_covariance[0] = 0.01;
      imuMsg.linear_acceleration_covariance[4] = 0.01;
      imuMsg.linear_acceleration_covariance[8] = 0.01;

      imuPub.publish(&imuMsg);

      myIMU.count = millis();
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
