/**
   FrSky control for MrBaddeley's D-O V2

   Author: Patrick Ryan, pat.m.ryan@gmail.com

   Original attribution:
   Sketch written by Reinhard Stockinger 2020/11
   Sketch is for the Printed Droid D-O Control PCB developed by Nitewing
   Latest skecth can always be found on www.printed-droid.com

   The following libraries are required:
   https://github.com/tockn/MPU6050_tockn
   https://github.com/DFRobot/DFRobotDFPlayerMini
   https://github.com/bolderflight/sbus
   https://github.com/ArminJo/ServoEasing

 **/


/**
   Channel mapping
   0: Throttle 1
   1: Throttle 2
   2: Not Mapped
   3: Nod angle (manual control)
   4: Cut Motors
   5: Head twist
   6: Momentary sound trigger
   7: Sound selection
   8: Head tilt
   9: Nothing
   10: IMU Nod control
   11: IMU Head Twist
   12: Volume knob (not working currently)
   13: Manual control toggle switch
*/

#include "Arduino.h"
#include <Wire.h>
#include <ServoEasing.h>
#include <MPU6050_tockn.h>
#include "sbus.h"
#include "DFRobotDFPlayerMini.h"

///////////////////////////////
// Configuration
///////////////////////////////

// RC
#define RC_MIN 172
#define RC_MAX 1811

// Channels start at 0 in the Arduino so map them to the corresponding OpenTX Mix Channel
#define CH1 0
#define CH2 1
#define CH3 2
#define CH4 3
#define CH5 4
#define CH6 5
#define CH7 6
#define CH8 7
#define CH9 8
#define CH10 9
#define CH11 10
#define CH12 11
#define CH13 12
#define CH14 13

// Servo Pins
#define SERVO_PIN_MAINBAR 0
#define SERVO_PIN_NOD 5
#define SERVO_PIN_TWIST 6
#define SERVO_PIN_TILT 1

// Motor controller pins
#define DIR1 13
#define PWM1 12
#define DIR2 11
#define PWM2 10

// PID constants
#define PID_P 25
#define PID_I 0
#define PID_D 0.8

///////////////////////////////

// MPU
MPU6050 mpu6050(Wire);

// SBUS
SbusRx sbus_rx(&Serial1);
bool failsafe;
bool lostFrame;

// Sound
DFRobotDFPlayerMini myDFPlayer;
int currentSound = 1;
int soundTimer;
bool soundPlaying = false;
int volume = 30;

// Servos
ServoEasing servoNod;
ServoEasing servoTwist;
ServoEasing servoTilt;
ServoEasing servoMainBar;

// Servo values
int nodAngle = 90;
int twistAngle = 90;
int tiltAngle = 90;
int mainBarAngle = 85;

// PID
float elapsedTime, time, timePrev;
float PID, error, previous_error;
float pid_p = 20;
float pid_i = 20;
float pid_d = 0;
int gyroXOffset = -0.3;

// Motor speed/direction
int mspeed = 10;
int motorspeed1 = 0;
int motordirection1 = HIGH;
int motorspeed2 = 0;
int motordirection2 = HIGH;

// Control Mode
// TODO: animations

// Mode 0: Control with IMU assistance for head movements
// Mode 1: Manual control of head movements
// Mode 2: Animation mode

// @deprecate
bool manualControl = true;

/*****************************************************************************/

void setup()
{
  Wire.begin();
  sbus_rx.Begin();
  Serial2.begin(9600);

  // Motor controller output pins
  pinMode(DIR1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM2, OUTPUT);

  // Attach servos
  servoMainBar.attach(SERVO_PIN_MAINBAR);
  servoNod.attach(SERVO_PIN_NOD);
  servoTwist.attach(SERVO_PIN_TWIST);
  servoTilt.attach(SERVO_PIN_TILT);

  // Center all servos
  servoMainBar.write(85);
  servoNod.writeMicroseconds(1500);
  servoTwist.writeMicroseconds(1500);
  servoTilt.writeMicroseconds(1500);

  // Wait for servos to center
  delay(1000);

  // MPU 6050
  mpu6050.begin();
  // NOTE: Only use this if you KNOW that D-O is properly centered when you turn it on. Otherwise, you should
  // use the `gyroXOffset` value.
  //  mpu6050.calcGyroOffsets();
  // gyroXOffset = mpu6050.

  // Set up DFPlayer
  if (!myDFPlayer.begin(Serial2, false))
  {
    while (true)
      ;
  }
  myDFPlayer.volume(volume);
  myDFPlayer.play(1);
  delay(1000);

  time = millis();
}

/*****************************************************************************/

void debug_rc_inputs()
{
//  Serial.print(sbus_rx.rx_channels()[CH1]);
//  Serial.print("\t");
//  Serial.print(sbus_rx.rx_channels()[CH2]);
//  Serial.print("\t");
//  Serial.print(sbus_rx.rx_channels()[CH3]);
//  Serial.print("\t");
//  Serial.print(sbus_rx.rx_channels()[CH4]);
//  Serial.print("\t");
//  Serial.print(sbus_rx.rx_channels()[CH5]);
//  Serial.print("\t");
//  Serial.print(sbus_rx.rx_channels()[CH6]);
//  Serial.print("\t");
//  Serial.print(sbus_rx.rx_channels()[CH7]);
//  Serial.print("\t");
//  Serial.print(sbus_rx.rx_channels()[CH8]);
//  Serial.print("\t");
//  Serial.print(sbus_rx.rx_channels()[CH9]);
//  Serial.print("\t");
//  Serial.print(sbus_rx.rx_channels()[CH10]);
//  Serial.print("\t");
//  Serial.print(sbus_rx.rx_channels()[CH11]);
//  Serial.print("\t");
//  Serial.print(sbus_rx.rx_channels()[CH12]);
//  Serial.print("\t");
//  Serial.println(sbus_rx.rx_channels()[CH13]);
}

/*****************************************************************************/

void loop()
{
  if (sbus_rx.Read())
  {
//    debug_rc_inputs();
    failsafe = sbus_rx.failsafe();

    if (failsafe)
    {
      handleFailsafe();
      return;
    }

    timePrev = time;
    time = millis();
    elapsedTime = (time - timePrev) / 1000;

    mpu6050.update();

    // in = RC_MAX, away = RC_MIN
    // TODO: change to a 3-way switch (IMU, manual, animation)
//    manualControl = sbus_rx.rx_channels()[CH5] > 1000;

    // RC Input
    if (disableMotors())
    {
      motorspeed1 = 0;
      motorspeed2 = 0;
    }
    else
    {
      motorspeed1 = map(sbus_rx.rx_channels()[0], RC_MIN, RC_MAX, 255, -255);
      motorspeed2 = map(sbus_rx.rx_channels()[1], RC_MIN, RC_MAX, -255, 255);
    }


    // PID Time
    error = mpu6050.getAngleX() - gyroXOffset;
    pid_p = PID_P * error;
    pid_i = pid_i + (PID_I * error);
    pid_d = PID_D * ((error - previous_error) / elapsedTime);
    PID = pid_p + pid_d;
    previous_error = error;

    // Mix the PID output with the motor speed
    mspeed = abs(PID);
    mspeed = map(mspeed, 0, 2100, 0, 700);
    if (mpu6050.getAngleX() < gyroXOffset)
    {
      mspeed = -mspeed;
    }
    if (mpu6050.getAngleX() > gyroXOffset)
    {
      mspeed = mspeed;
    }

    motorspeed1 = motorspeed1 + mspeed;
    motorspeed2 = motorspeed2 - mspeed;

    if (motorspeed1 < 0)
    {
      motordirection1 = LOW;
      motorspeed1 = -motorspeed1;
    }
    else if (motorspeed1 > 0)
    {
      motordirection1 = HIGH;
    }

    if (motorspeed2 < 0)
    {
      motordirection2 = LOW;
      motorspeed2 = -motorspeed2;
    }
    else if (motorspeed2 > 0)
    {
      motordirection2 = HIGH;
    }

    if (motorspeed1 > 254)
    {
      motorspeed1 = 255;
    }

    if (motorspeed2 > 254)
    {
      motorspeed2 = 255;
    }


    digitalWrite(DIR1, motordirection1);
    analogWrite(PWM1, motorspeed1);
    digitalWrite(DIR2, motordirection2);
    analogWrite(PWM2, motorspeed2);

    handleSound();
    handleServos();
  }
}

/*****************************************************************************/

bool disableMotors()
{
  return sbus_rx.rx_channels()[CH5] == RC_MIN;
}

/*****************************************************************************
  // Servos
******************************************************************************/
void handleServos()
{
  // Handle manual control of servos
  if (manualControl)
  {
    nodAngle = map(sbus_rx.rx_channels()[3], RC_MIN, RC_MAX, 1700, 1200);
    twistAngle = map(sbus_rx.rx_channels()[CH6], RC_MIN, RC_MAX, 2000, 1000);
  }
  else // IMU control
  {
    nodAngle = map(sbus_rx.rx_channels()[10], 283, 1700, 1200, 1700);
    twistAngle = map(sbus_rx.rx_channels()[11], RC_MIN, RC_MAX, 2000, 1000);
  }

  tiltAngle = map(sbus_rx.rx_channels()[8], RC_MIN, RC_MAX, 1200, 1700);
//  Serial.println(tiltAngle);
  mainBarAngle = map(sbus_rx.rx_channels()[10], 283, 1700, 1700, 1200);

  // Update Servos
  servoMainBar.writeMicroseconds(mainBarAngle);
  servoTwist.writeMicroseconds(twistAngle);
  servoTilt.writeMicroseconds(tiltAngle);
  servoNod.writeMicroseconds(nodAngle);
}

/*****************************************************************************
  // Handle sound selection and triggering
******************************************************************************/
void handleSound()
{
  // Volume
  // TODO: Seems to be a bug in the DFPlayer library where volume isn't set correctly outside
  // of the setup function.
  //    int newVolume = map(sbus_rx.rx_channels()[12], RC_MIN, RC_MAX, 0, 30);
  //    if (newVolume != volume)
  //    {
  //        myDFPlayer.volume(volume);
  //        volume = newVolume;
  //    }

  // Select sound group and pick a random sound
  if (sbus_rx.rx_channels()[7] < 500)
  {
    currentSound = random(3, 5);
  }
  else if (sbus_rx.rx_channels()[7] > 500 && sbus_rx.rx_channels()[7] < 1500)
  {
    currentSound = random(6, 10);
  }
  else
  {
    currentSound = random(11, 14);
  }

  // Trigger the sound
  if (sbus_rx.rx_channels()[6] > 1000 && !soundPlaying)
  {
    myDFPlayer.play(currentSound);
    soundPlaying = true;
    soundTimer = millis();
  }

  // Lockout for 5 seconds if playing
  if (soundPlaying && (millis() - soundTimer > 5000))
  {
    soundPlaying = false;
  }
}

/*****************************************************************************
   If we lose connetion or the transmitter shuts off then halt D-O and
   center all servos
 *****************************************************************************/
void handleFailsafe()
{
  // Halt
  analogWrite(PWM1, 0);
  analogWrite(PWM2, 0);

  // Center all servos
  servoMainBar.write(85);
  servoNod.write(90);
  servoTwist.write(90);
  servoTilt.write(90);
}
