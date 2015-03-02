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
int rc_command[3] = {0,0,0};                // Use CH_DRIVE and CH_STEER

//==== ENCODER ====//
Truck_Enc encoders;

int dist_travelled = 0;
int steeringAngle = 0;

//==== MOTOR DRIVERS ====//
Motor_Driver motorDrivers(&Serial2,&Serial3);
unsigned char drive_command = 0;
unsigned char steer_command = 0;

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
  Serial2.begin(2400);
  Serial3.begin(2400);
  Wire.begin(); //I2C
  
  //==== MOTOR DRIVER ====//
  // Make sure no command
  //motorDrivers.sendSteerCommand(steer_command);
  //motorDrivers.sendDriveCommand(drive_command);
  //======================//
  
  //==== MPU ====//
//  // Only necessary if using multiple IMU
//  MPU.selectDevice(DEVICE_TO_USE); 
//  // Start MPU
//  MPU.init(MPU_UPDATE_RATE, MPU_MAG_MIX_GYRO_AND_MAG,
//  MAG_UPDATE_RATE, MPU_LPF_RATE);
  //=============//
  
  //==== RC_PWM ====//
  pinMode(RC_PIN_CH1, INPUT); // Set our input pins as such
  pinMode(RC_PIN_CH2, INPUT);
  pinMode(RC_PIN_CH3, INPUT);
  //===============//
  
}

/*=================================Start Main Loop====================================================*/
void loop() {
  
  //==== MPU ====//
//  if (MPU.read()) {                                   // get the latest data if ready yet
//    mpu_orien = MPU.m_fusedEulerPose;
//    mpu_acl = MPU.m_calAccel;
//  }
  
  //writeAngle(RF_serial,mpu_orien);
  //writeAccl(RF_serial,mpu_acl);
  
  //MPU.printVector(MPU.m_calAccel);
  //=============//
  
  //==== RC_PWM ====//
  rc_command[0] = rc_read_pulse(RC_PIN_CH1);
  rc_command[CH_DRIVE] = rc_read_pulse(RC_PIN_CH2);
  rc_command[CH_STEER] = rc_read_pulse(RC_PIN_CH3);
  
  print_channel(&Serial, rc_command[0], rc_command[CH_DRIVE], rc_command[CH_STEER]);
  //===============//
  
  drive_command = pulse_to_command(rc_command[CH_DRIVE],CH_DRIVE);
  steer_command = pulse_to_command(rc_command[CH_STEER],CH_STEER);
  
  Serial.print("\nCommand:");Serial.print(int(drive_command));Serial.print(", ");Serial.println(int(steer_command));
  
  motorDrivers.sendDriveCommand(drive_command);
  motorDrivers.sendSteerCommand(steer_command);
  
  //dist_travelled = encoders.readDist();
  //steeringAngle = encoders.readAngle();
  
  //encoders.serialWriteVals(RF_serial);
  //Serial.println();
  
  //motorDrivers.sendSteerCommand(steer_command);
  //motorDrivers.sendDriveCommand(drive_command);
  
//  unsigned char m1b = 30;
//  unsigned char m1n = 64;
//  unsigned char m1f = 127;
//  
//  unsigned char m2b = 30+127;
//  unsigned char m2n = 64+127;
//  unsigned char m2f = 127+127;
//  
//  motorDrivers.sendDriveCommand(m2b);
//  delay(3000);
//  motorDrivers.sendDriveCommand(m2n);
//  delay(3000);
//  motorDrivers.sendDriveCommand(m2f);
//  delay(3000);
//  motorDrivers.sendSteerCommand(m1b);
//  delay(3000);
//  motorDrivers.sendSteerCommand(m1n);
//  delay(3000);
//  motorDrivers.sendSteerCommand(m1f);
//  delay(3000);
  
  
//  Serial2.write(m1b);
//  Serial.print("Back M1");Serial.write(m1b);Serial.println();
//  delay(3000);
//  Serial2.write(m1n);
//  Serial.print("Neut M1");Serial.write(m1n);Serial.println();
//  delay(3000);
//  Serial2.write(m1f);
//  Serial.print("Forw M1");Serial.write(m1f);Serial.println();
//  delay(3000);
//  Serial2.write(m2b);
//  Serial.print("Back M2");Serial.write(m2b);Serial.println();
//  delay(3000);
//  Serial2.write(m2n);
//  Serial.print("Neut M2");Serial.write(m2n);Serial.println();
//  delay(3000);
//  Serial2.write(m2f);
//  Serial.print("Forw M2");Serial.write(m2f);Serial.println();
//  delay(3000);
 
  delay(DELAY_T); // May not be necessary if there is more going on here
}
