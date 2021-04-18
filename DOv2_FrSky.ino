/**
 * FrSky control for MrBaddeley's D-O V2
 * 
 * Author: Patrick Ryan, pat.m.ryan@gmail.com
 * 
 * Original attribution:
 * Sketch written by Reinhard Stockinger 2020/11
 * Sketch is for the Printed Droid D-O Control PCB developed by Nitewing
 * Latest skecth can always be found on www.printed-droid.com
 * 
 * The following libraries are required:
 * https://github.com/tockn/MPU6050_tockn
 * https://github.com/DFRobot/DFRobotDFPlayerMini
 * https://github.com/bolderflight/sbus
 * 
 **/

#include "Arduino.h"
#include <Wire.h>
#include <Servo.h>
#include <MPU6050_tockn.h>
#include "sbus.h"
#include "DFRobotDFPlayerMini.h"

///////////////////////////////
// Configuration

// RC
#define RC_MIN 172
#define RC_MAX 1811

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

// Rolling input average
#define ROLLING_AVERAGE 6

///////////////////////////////

// MPU
MPU6050 mpu6050(Wire);

// SBUS
SbusRx sbus_rx(&Serial1);
bool failsafe;
bool lostFrame;

// Sound
DFRobotDFPlayerMini myDFPlayer;
int currentsound = 1;
int soundTimer;
bool soundPlaying = false;

// Servos
Servo ServoNod;
Servo ServoTwist;
Servo ServoTilt;
Servo ServoMainBar;

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

bool mode = 0;

// @deprecate
bool manualControl = false;

// Rolling average
// NOTE: This isn't necessary for the main bar
int tiltValues[ROLLING_AVERAGE];
int twistValues[ROLLING_AVERAGE];
int nodValues[ROLLING_AVERAGE];
int tiltAverage = 0;
int twistAverage = 0;
int nodAverage = 0;
int rollingAverageIndex = 0;

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
    ServoMainBar.attach(SERVO_PIN_MAINBAR);
    ServoNod.attach(SERVO_PIN_NOD);
    ServoTwist.attach(SERVO_PIN_TWIST);
    ServoTilt.attach(SERVO_PIN_TILT);

    // Center all servos
    ServoMainBar.write(90);
    ServoNod.write(90);
    ServoTwist.write(90);
    ServoTilt.write(90);

    // Wait for servos to center
    delay(1000);

    // MPU 6050
    mpu6050.begin();
    // NOTE: Only use this if you KNOW that D-O is properly centered when you turn it on. Otherwise, you should
    // use the `gyroXOffset` value.
    //  mpu6050.calcGyroOffsets();
    // gyroXOffset = mpu6050.

    // Set up DFPlayer
    if (!myDFPlayer.begin(Serial2))
    {
        while (true)
            ;
    }
    Serial.println(F("DFPlayer Mini online."));
    myDFPlayer.volume(5);
    delay(1000);
    myDFPlayer.play(1);
    delay(3000);

    time = millis();
}

void loop()
{
    if (sbus_rx.Read())
    {
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
        manualControl = sbus_rx.rx_channels()[4] > 1000;

        // RC Input
        motorspeed1 = map(sbus_rx.rx_channels()[0], RC_MIN, RC_MAX, 255, -255);
        motorspeed2 = map(sbus_rx.rx_channels()[1], RC_MIN, RC_MAX, -255, 255);

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

void handleSound()
{
    static unsigned long timer = millis();

    // Volume
    int vol = map(sbus_rx.rx_channels()[7], RC_MIN, RC_MAX, 0, 30);
    myDFPlayer.volume(vol);

    // Select sound group and pick a random sound
    if (sbus_rx.rx_channels()[7] < 500)
    {
        currentsound = random(3, 5);
    }
    else if (sbus_rx.rx_channels()[7] > 500 && sbus_rx.rx_channels()[7] < 1500)
    {
        currentsound = random(6, 10);
    }
    else
    {
        currentsound = random(11, 14);
    }

    // Trigger the sound
    if (sbus_rx.rx_channels()[6] > 1000 && !soundPlaying)
    {
        myDFPlayer.play(currentsound);
        soundPlaying = true;
        soundTimer = millis();
    }

    // Lockout for 5 seconds if playing
    if (soundPlaying && (millis() - soundTimer > 5000))
    {
        soundPlaying = false;
    }
}

void handleServos()
{
    // Handle manual control of servos
    if (manualControl)
    {
        nodValues[rollingAverageIndex] = map(sbus_rx.rx_channels()[9], RC_MIN, RC_MAX, 40, 140);
        twistValues[rollingAverageIndex] = map(sbus_rx.rx_channels()[2], RC_MIN, RC_MAX, 140, 40);
    }
    else
    {
        nodValues[rollingAverageIndex] = map(sbus_rx.rx_channels()[3], RC_MIN, RC_MAX, 40, 140);
        twistValues[rollingAverageIndex] = map(sbus_rx.rx_channels()[11], RC_MIN, RC_MAX, 140, 40);
    }

    tiltValues[rollingAverageIndex] = map(sbus_rx.rx_channels()[8], RC_MIN, RC_MAX, 40, 140);

    updateInputAverages();

    rollingAverageIndex++;

    if (rollingAverageIndex >= ROLLING_AVERAGE)
    {
        rollingAverageIndex = 0;
    }

    ServoMainBar.write(map(sbus_rx.rx_channels()[10], RC_MIN, RC_MAX, 120, 60));
    ServoTwist.write(twistAverage);
    ServoTilt.write(tiltAverage);
    ServoNod.write(nodAverage);
}

void updateInputAverages()
{
    for (int i = 0; i < ROLLING_AVERAGE; i++)
    {
        tiltAverage += tiltValues[i];
        twistAverage += twistValues[i];
        nodAverage += nodValues[i];
    }
    tiltAverage = tiltAverage / ROLLING_AVERAGE;
    twistAverage = twistAverage / ROLLING_AVERAGE;
    nodAverage = nodAverage / ROLLING_AVERAGE;
}

/**
 * If we lose connetion or the transmitter shuts off then halt D-O and
 * center all servos
 **/
void handleFailsafe()
{
    // Halt
    analogWrite(PWM1, 0);
    analogWrite(PWM2, 0);

    // Center all servos
    ServoMainBar.write(90);
    ServoNod.write(90);
    ServoTwist.write(90);
    ServoTilt.write(90);
}