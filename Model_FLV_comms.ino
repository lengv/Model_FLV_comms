/*
  Created: 2015-02-15 [yyyy-mm-dd]

 */

#define DEBUG_ON
#define WIFI_ON
#include <String.h>

// Communication
#include <WiFi.h>
//#include <SoftwareSerial.h> //For SD card
#include <SPI.h> // WIFI shield requires, CAN bus shield requires
//#include <Wire.h> // I2C

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

// Encoders
#include <Encoder.h>
#include "truck_encoder.h"

// RC
#include "RC.h"

// Motor Driver
#include "MotorDriver.h"

/*=================================Defines and Globals===============================================*/
//==== CONTROL DEFINES - Comment out if not connected
#define MPU_ON
#define MOTORS_ON

//==== MPU ====//
#define DEVICE_TO_USE    0
#define MPU_ACCL_FSR MPU_ACCL_FSR_2 // Note this is default - can set value to 2/4/8/16

MPU9150Lib MPU;                             // the MPU object
float* mpu_orien;
short* mpu_acl;

boolean mpu_con = false;

//==== RC_PWM ====//
int rc_command[3] = {0, 0, 0};              // Use CH_DRIVE and CH_STEER

//==== ENCODER ====//
Truck_Enc encoders;

float dist_travelled = 0;
float steeringAngle = 0;

//==== MOTOR DRIVERS ====//
Motor_Driver motorDrivers(&Serial2, &Serial3);
unsigned char drive_command = 0;
unsigned char steer_command = 0;
unsigned char tmp_drive_command = 0;
unsigned char tmp_steer_command = 0;

//==== RF ====//
HardwareSerial *RF_serial = &Serial1;

//==== WIFI ====//
char ssid[] = "4360D-114771-M";      // your network SSID (name)

// Standard ports
int telnet_ = 23;
int web_ = 80;

int status = WL_IDLE_STATUS;

WiFiServer server(web_);

//==== GENERAL ====//
// Timer for timestamps
unsigned long timer;

//  BAUDRATE defines the speed to use for the debug serial port. Note: if this is changed, the RF comm (ACP-220) must be reprogrammed
#define  BAUDRATE  57600

// Delay time to allow RF to work
#define DELAY_T 10 //ms

String data_line; // Accumulator

int wifi_count=0;

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

/*=====================================Setup==========================================================*/
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(BAUDRATE);
  Serial1.begin(BAUDRATE);
  Serial2.begin(MOTOR_DRIVER_BAUDRATE);
  Serial3.begin(MOTOR_DRIVER_BAUDRATE);
  Wire.begin(); //I2C
  #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega328P__)
    // deactivate internal pull-ups for twi
    // as per note from atmega8 manual pg167
    cbi(PORTC, 4);
    cbi(PORTC, 5);
  #else
    // deactivate internal pull-ups for twi
    // as per note from atmega128 manual pg204
    cbi(PORTD, 0);
    cbi(PORTD, 1);
  #endif

  //==== MOTOR DRIVER ====//
#ifdef MOTOR_ON
  // Make sure no command
  motorDrivers.sendSteerCommand(steer_command);
  motorDrivers.sendDriveCommand(drive_command);
#endif
  //======================//
  //==== MPU ====//
#ifdef MPU_ON
  // Only necessary if using multiple IMU
  MPU.selectDevice(DEVICE_TO_USE);
  // Start MPU
  mpu_con = MPU.init(MPU_UPDATE_RATE, MPU_MAG_MIX_GYRO_AND_SOME_MAG,
                     MAG_UPDATE_RATE, MPU_LPF_RATE);
  mpu_set_accel_fsr(MPU_ACCL_FSR);
#endif
  //=============//
  //==== RC_PWM ====//
  pinMode(RC_PIN_CH1, INPUT); // Set our input pins as such
  pinMode(RC_PIN_CH2, INPUT);
  pinMode(RC_PIN_CH3, INPUT);
  //===============//
  //==== WIFI ====//
  #ifdef WIFI_ON
    if (WiFi.status() == WL_NO_SHIELD) {
      Serial.println("WiFi shield not present");
      // don't continue:
      while (true);
    }
    while ( status != WL_CONNECTED && wifi_count <1 ) {
      Serial.print("Attempting to connect to SSID: ");
      Serial.println(ssid);
      // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
      status = WiFi.begin(ssid);
  
      // wait for connection:
      delay(100);
      wifi_count++;
    }
    server.begin();
    printWifiStatus();
  #endif
  //===============//

  // Reserve some space for the string
  data_line.reserve(100);
}

//int maxX = -9999;
//int minX = 9999;
//int maxZ = -9999;
//int minZ = 9999;
//int maxY = -9999;
//int minY = 9999;

//int change = 0;
//int _count = 1;
//int _skip = 20;

/*=================================Start Main Loop====================================================*/
void loop() {
  
  #ifdef WIFI_ON
    WiFiClient client = server.available();
  #endif
  
  //Serial.println("1");
  timer = millis();
  //Serial.println("2");

  //==== MPU ====//
  Wire.beginTransmission(104); //check if mpu is there
  if(Wire.endTransmission() == 0){
    if (mpu_con && MPU.read()) {                                   // get the latest data if ready yet
      mpu_orien = MPU.m_fusedEulerPose;
      mpu_acl = MPU.m_calAccel;
    }
  } else{
//    mpu_con = MPU.init(MPU_UPDATE_RATE, MPU_MAG_MIX_GYRO_AND_SOME_MAG,
//                     MAG_UPDATE_RATE, MPU_LPF_RATE);
//    mpu_set_accel_fsr(MPU_ACCL_FSR);
    Serial.println("no IMU");
  }
  //Serial.println("3");
  //  if(_count > _skip){
  //    if(mpu_acl[VEC3_X] < minX) {minX=mpu_acl[VEC3_X];change = 1;}
  //    if(mpu_acl[VEC3_X] > maxX) {maxX=mpu_acl[VEC3_X];change = 1;}
  //    if(mpu_acl[VEC3_Y] < minY) {minY=mpu_acl[VEC3_Y];change = 1;}
  //    if(mpu_acl[VEC3_Y] > maxY) {maxY=mpu_acl[VEC3_Y];change = 1;}
  //    if(mpu_acl[VEC3_Z] < minZ) {minZ=mpu_acl[VEC3_Z];change = 1;}
  //    if(mpu_acl[VEC3_Z] > maxZ) {maxZ=mpu_acl[VEC3_Z];change = 1;}
  //  }


  //  if(_count < _skip + 2){
  //  _count++;
  //    writeAngle(&Serial,mpu_orien);
  //    writeAccl(&Serial,mpu_acl);
  //    Serial.println();
  //  }
  //
  //  if(change){
  //    Serial.print("X:");Serial.print(minX);Serial.print(',');Serial.print(maxX);
  //    Serial.print(",Y:");Serial.print(minY);Serial.print(',');Serial.print(maxY);
  //    Serial.print(",Z:");Serial.print(minZ);Serial.print(',');Serial.print(maxZ);
  //    Serial.println();
  //    change = 0;
  //  }
  //MPU.printVector(MPU.m_calAccel);
  //=============//

  //==== RC_PWM ====//
  rc_command[0] = rc_read_pulse(RC_PIN_CH1);
  rc_command[CH_DRIVE] = rc_read_pulse(RC_DRIVE_PIN);
  rc_command[CH_STEER] = rc_read_pulse(RC_STEER_PIN);
  //Serial.println("4");
  //print_channel(RF_serial, rc_command[0], rc_command[CH_DRIVE], rc_command[CH_STEER]);
  //===============//


  if (rc_command[CH_DRIVE] <= ch_max[CH_DRIVE] && rc_command[CH_DRIVE] >= ch_min[CH_DRIVE]) {
    drive_command = pulse_to_command(rc_command[CH_DRIVE], CH_DRIVE);
    steer_command = pulse_to_command(rc_command[CH_STEER], CH_STEER);
  } else {
    Serial.println("no input");
    if (RF_serial->available() == 2) {
      drive_command = RF_serial->read();
      steer_command = RF_serial->read();
      Serial.println("from MATLAB");
    } else{
      drive_command = 0;
      steer_command = 0;
    }
  }

  if (RF_serial->available() == 2) {
    drive_command = RF_serial->read();
    steer_command = RF_serial->read();
    Serial.println("from MATLAB");
  }
  //Serial.println("5");

  // Read values
  encoders.readEnc();

  // Get values from object
  dist_travelled = encoders.getDist();
  steeringAngle = encoders.getAngle();

  //Serial.print("[Pulse:"); Serial.print(encoders.getEncAbs()); Serial.println(']');
  //Serial.println("6");

  // Drive command section
#ifdef MOTORS_ON
  motorDrivers.sendDriveCommand(drive_command);

  // Prevent wheel from turning too far, allowing rotation in opposite direction
  // Note: that if the angular velocity is high the wheel may go beyond the bound
  if (steeringAngle <= PI && steeringAngle >= PI_ON_2 && steer_command < STEER_ZR) {
    steer_command = STEER_ZR;
    //Serial.println("Not sending steering1");
  } else if (steeringAngle <= PI_ON_2_3 && steeringAngle > PI && steer_command > STEER_ZR) {
    steer_command = STEER_ZR;
    //Serial.println("Not sending steering2");
  }

  // Force steer angle to zero
  // TODO

  //steer_command = pulse_to_position_command(rc_command[CH_STEER], steeringAngle,CH_STEER);

  motorDrivers.sendSteerCommand(steer_command);

#endif // MOTORS_ON
  //Serial.println("7");

  //Serial.print('['); Serial.print(timer); Serial.print(']');
  //RF_serial->print('[');RF_serial->print(timer);RF_serial->print(']');
  //encoders.serialWriteVals(RF_serial);
  //encoders.serialWriteVals(&Serial);
  //RF_serial->print("[Command:");RF_serial->print(drive_command);RF_serial->print(','); RF_serial->print(steer_command);RF_serial->print(']');
//  Serial.println(); Serial.print("Command:"); Serial.print(drive_command); Serial.print(','); Serial.println(steer_command);
//  writeAngle(RF_serial, mpu_orien);
//  writeAccl(RF_serial, mpu_acl);
//
//  writeAngle(&Serial, mpu_orien);
//  writeAccl(&Serial, mpu_acl);
  //writeAcclVal(&Serial,mpu_acl);
    data_line = "";
    data_line += "[";
    data_line += timer;
    data_line += "]";
    data_line += "[o:";
    data_line += mpu_orien[VEC3_X] * RAD_TO_DEGREE;
    data_line += ",";
    data_line += mpu_orien[VEC3_Y] * RAD_TO_DEGREE;
    data_line += ",";
    data_line += mpu_orien[VEC3_Z] * RAD_TO_DEGREE;
    data_line += "]";
    data_line += "[a:";
    data_line += mpu_acl[VEC3_X] * ACCL_G_PER_VAL_X * GRAVITY;
    data_line += ",";
    data_line += mpu_acl[VEC3_Y] * ACCL_G_PER_VAL_Y * GRAVITY;
    data_line += ",";
    data_line += mpu_acl[VEC3_Z] * ACCL_G_PER_VAL_Z * GRAVITY;
    data_line += "]";
    data_line += "[e:";
    data_line += dist_travelled;
    data_line += ",";
    data_line += steeringAngle;
    data_line += "]";
    data_line += "[c:";
    data_line += drive_command;
    data_line += ",";
    data_line += steer_command;
    data_line += "]";
    data_line += "[w:";
    data_line += MPU.m_rawGyro[VEC3_X];
    data_line += ",";
    data_line += MPU.m_rawGyro[VEC3_Y];
    data_line += ",";
    data_line += MPU.m_rawGyro[VEC3_Z];
    data_line += "]\n";
    
    #ifdef WIFI_ON
      if (client.connected()) {
    //    client.print('['); client.print(timer); client.print(']');
    //    writeAngle(&client, mpu_orien);
    //    writeAccl(&client, mpu_acl);
    //    encoders.serialWriteVals(&client);
    //    client.println();
    
        
        //Serial.println("Sending data");
        client.print(data_line);
        
      }
    #endif
  //encoders.serialWriteRaw(&Serial);
  Serial.print(data_line);
  
//  RF_serial->println();
  Serial.println();

  delay(DELAY_T); // May not be necessary if there is more going on here
  //Serial.println("8");
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
