/*
  Created: 2015-02-15 [yyyy-mm-dd]
  
 */
 
#define DEBUG_ON
 
// MPU
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU9150Lib.h"
#include "CalLib.h"
#include <dmpKey.h>
#include <dmpmap.h>
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
#include <EEPROM.h>
#include "mpu.h"

// RC
#include "RC.h"

// Encoders
#include <Encoder.h>
#include "truck_encoder.h"

// Motor Driver
#include "MotorDriver.h"

/*=================================Defines and Globals===============================================*/
//==== MPU ====//
#define  DEVICE_TO_USE    0

MPU9150Lib MPU;                             // the MPU object
float* mpu_orien;
short* mpu_acl;

//==== RC_PWM ====//
int rc_ch1; // Here's where we'll keep our channel values
int rc_ch2;
int rc_ch3;

//==== ENCODER ====//
Truck_Enc encoders;

int dist_travelled = 0;
int steeringAngle = 0;

//==== MOTOR DRIVERS ====//
Motor_Driver motorDrivers(&Serial2,&Serial3);
char drive_command = 0;
char steer_command = 0;

//==== RF ====//
HardwareSerial *RF_serial = &Serial1;

//==== GENERAL ====//
//  BAUDRATE defines the speed to use for the debug serial port. Note: if this is changed, the RF comm (ACP-220) must be reprogrammed
#define  BAUDRATE  57600 

// Delay time to allow RF to work
#define DELAY_T 20 //ms

/*=====================================Setup==========================================================*/
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(BAUDRATE);
  Serial1.begin(BAUDRATE);
  Wire.begin(); //I2C
  
  //==== MOTOR DRIVER ====//
  // Make sure no command
  //motorDrivers.sendSteerCommand(steer_command);
  //motorDrivers.sendDriveCommand(drive_command);
  //======================//
  
  //==== MPU ====//
  // Only necessary if using multiple IMU
  MPU.selectDevice(DEVICE_TO_USE); 
  // Start MPU
  MPU.init(MPU_UPDATE_RATE, MPU_MAG_MIX_GYRO_AND_MAG,
  MAG_UPDATE_RATE, MPU_LPF_RATE);
  //=============//
  
  //==== RC_PWM ====//
  pinMode(rc_ch1, INPUT); // Set our input pins as such
  pinMode(rc_ch2, INPUT);
  pinMode(rc_ch3, INPUT);
  //===============//
  
}

/*=================================Start Main Loop====================================================*/
void loop() {
  
  //==== MPU ====//
  if (MPU.read()) {                                   // get the latest data if ready yet
    mpu_orien = MPU.m_fusedEulerPose;
    mpu_acl = MPU.m_calAccel;
  }
  
  writeAngle(RF_serial,mpu_orien);
  writeAccl(RF_serial,mpu_acl);
  
  //MPU.printVector(MPU.m_calAccel);
  //=============//
  
  //==== RC_PWM ====//
  rc_ch1 = rc_read_pulse(RC_PIN_CH1);
  rc_ch2 = rc_read_pulse(RC_PIN_CH2);
  rc_ch3 = rc_read_pulse(RC_PIN_CH3);
  
  print_channel(RF_serial, rc_ch1, rc_ch2, rc_ch3);
  //===============//
  
  dist_travelled = encoders.readDist();
  steeringAngle = encoders.readAngle();
  
  encoders.serialWriteVals(RF_serial);
  //Serial.println();
  
  //motorDrivers.sendSteerCommand(steer_command);
  //motorDrivers.sendDriveCommand(drive_command);
 
  delay(DELAY_T); // May not be necessary if there is more going on here
}
