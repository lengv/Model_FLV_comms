/*
  Created: 2015-02-15 [yyyy-mm-dd]

 */
 
//==== CONTROL DEFINES - Comment out if not connected
#define MPU_ON
#define MOTORS_ON
//#define DEBUG_ON  // Add serial out onto Arduino display
//#define WIFI_ON
#define RF_ON
#define ENC_ON
//#define RC_ON
//#define RF_READ_ON
#define PID_ON

#define CSV_MODE  // Comment for readable output

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

// Load sensor
#include <HX711.h>
#include "load_sensor.h"

// PID
#include <PID_v1.h>

/*=================================Defines and Globals===============================================*/


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
Motor_Driver motorDrivers(&Serial2, &Serial2);
unsigned char drive_command = 0;
unsigned char steer_command = 0;
unsigned char tmp_drive_command = 0;
unsigned char tmp_steer_command = 0;

//==== RF ====//
HardwareSerial *RF_serial = &Serial1;

//==== Load Sensor ====//
HX711 load_right(22,23); // Right load sensor
HX711 load_left(24,25); // Left load sensor
HX711 load_rear(26,27); // Rear load sensor

float mass_right = 0;
float mass_left = 0;
float mass_rear = 0;

//==== PID ====//
#ifdef PID_ON
  double Setpoint_d, Input_d, Output_d;
  double Setpoint_s, Input_s, Output_s;
  //Specify the links and initial tuning parameters
  double Kp_d=1, Ki_d=0, Kd_d=0;
  double Kp_s=100, Ki_s=1, Kd_s=0;
  
  PID drive_controller(&Input_d, &Output_d, &Setpoint_d, Kp_d, Ki_d, Kd_d, DIRECT);
  PID steer_controller(&Input_s, &Output_s, &Setpoint_s, Kp_s, Ki_s, Kd_s, DIRECT);
#endif

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
#define  BAUDRATE  19200

// Delay time to allow RF to work
#define DELAY_T 10 //ms

String data_line = ""; // Accumulator

int wifi_count=0;

int drive_temp=0;

int initialising_alpha = 1;

// For deactivating internal pull-ups 
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
  mpu_con = MPU.init(MPU_UPDATE_RATE, MPU_MAG_MIX_GYRO_AND_MAG,
                     MAG_UPDATE_RATE, MPU_LPF_RATE);
  mpu_set_accel_fsr(MPU_ACCL_FSR);
#endif
  //=============//
  //==== RC_PWM ====//
  pinMode(RC_PIN_CH1, INPUT); // Set our input pins as such
  pinMode(RC_PIN_CH2, INPUT);
  pinMode(RC_PIN_CH3, INPUT);
  //===============//
  
  //==== PID ====//
  #ifdef PID_ON
    drive_controller.SetSampleTime(50);
    steer_controller.SetSampleTime(50);
    
    //drive_controller.SetOutputLimits(-64, 63);
    drive_controller.SetOutputLimits(-10, 10);
    steer_controller.SetOutputLimits(-63, 63);
    
    Setpoint_d = 0;
    Setpoint_s = 0;
    
    drive_controller.SetMode(MANUAL);
    steer_controller.SetMode(AUTOMATIC);
    
    drive_command = DRIVE_ZR;
    steer_command = STEER_ZR;
  #endif
  //==============//
  
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
  init_load_sensors(&load_right, &load_left, &load_rear);
  
  // Reserve some space for the string
  data_line.reserve(100);
  
  delay(2000);
}

/*=================================Start Main Loop====================================================*/
void loop() {
  
  #ifdef WIFI_ON
    WiFiClient client = server.available();
  #endif
  
  //Serial.println("1");
  timer = millis();
  //Serial.println("2");

  //==== MPU ====//
//  Wire.beginTransmission(104); //check if mpu is there
//  if(Wire.endTransmission() == 0){
    if (mpu_con && MPU.read()) {                                   // get the latest data if ready yet
      mpu_orien = MPU.m_fusedEulerPose;
      mpu_acl = MPU.m_calAccel;
    }
//  } else{
//    Serial.println("no IMU");
//  }
  //=============//

  //==== RC_PWM ====//
  rc_command[0] = rc_read_pulse(RC_PIN_CH1);
  rc_command[CH_DRIVE] = rc_read_pulse(RC_DRIVE_PIN);
  rc_command[CH_STEER] = rc_read_pulse(RC_STEER_PIN);
  //Serial.println("4");
  //print_channel(RF_serial, rc_command[0], rc_command[CH_DRIVE], rc_command[CH_STEER]);
  //===============//

  #ifdef RC_ON
    if (rc_command[CH_DRIVE] == 0){
      Serial.println("no RC input");
      if (RF_serial->available() == 2) {
        drive_command = RF_serial->read();
        steer_command = RF_serial->read();
        Serial.println("from MATLAB");
      } else{
        drive_command = 0;
        steer_command = 0;
      }
    } else{
      if(rc_command[CH_DRIVE] > ch_max[CH_DRIVE]){
        rc_command[CH_DRIVE] = ch_max[CH_DRIVE];
      } else if(rc_command[CH_DRIVE] < ch_min[CH_DRIVE]){
         rc_command[CH_DRIVE] = ch_min[CH_DRIVE];
      }
      
      drive_command = pulse_to_command(rc_command[CH_DRIVE], CH_DRIVE);
      steer_command = pulse_to_command(rc_command[CH_STEER], CH_STEER);
    }
  #endif RC_ON

  #ifdef RF_READ_ON
    if (RF_serial->available() == 2) {
      drive_command = RF_serial->read();
      steer_command = RF_serial->read();
      Serial.println("from MATLAB");
    }
  #endif
  
  
  // Read values
  encoders.readEnc();

  // Get values from object
  dist_travelled = encoders.getDist();
  steeringAngle = encoders.getAngle();

  #ifdef PID_ON
//    if(dist_travelled <= 2000){
    if(dist_travelled >= -1000){
      
      if(initialising_alpha && abs(steeringAngle - 0) > PI/20){
        // Set wheel to 0
        Setpoint_d = 0;
        Setpoint_s = 0;
      }else{
        initialising_alpha = 0;
        // Forward
        Setpoint_d = 80;
        Setpoint_s = 0;
        
        drive_command = DRIVE_FR;// Temporary while drive PID is set to manual
      }
//    }else if(dist_travelled <= 2000+1000){
//      // Turn
//      Setpoint_d = 80;
//      Setpoint_s = PI/3;
    }else{
      // Stop
      Setpoint_d = 0;
      Setpoint_s = 0;
      drive_command = DRIVE_ZR; // Temporary while drive PID is set to manual
      Serial.println("stop!");
    }
    
    Input_d = encoders.getVel();
    Input_s = steeringAngle;
    
//    drive_temp = drive_command + Output_d;
    drive_temp = drive_command;
    
//    if(drive_temp < DRIVE_FR){
//      drive_temp = DRIVE_FR;
//    } else if(drive_temp > DRIVE_FF){
//      drive_temp = DRIVE_FF;
//    }
    
//    drive_controller.Compute();
    steer_controller.Compute();
    
    drive_command = drive_temp;
    steer_command = (Output_s + STEER_ZR);
    
//    Serial.print("d/s:"); Serial.print(Output_d); Serial.print(","); Serial.print(Output_s);
//    Serial.print(" enc:");Serial.print(dist_travelled);
//    Serial.print(","); Serial.print(steeringAngle);
//    Serial.print(" vel:");Serial.print(encoders.getVel());
//    Serial.println();
  #endif

  // Drive command section
  #ifdef MOTORS_ON
    motorDrivers.sendDriveCommand(drive_command);
  
    // Prevent wheel from turning too far, allowing rotation in opposite direction
    // Note: that if the angular velocity is high the wheel may go beyond the bound
    if (steeringAngle <= PI && steeringAngle >= PI_ON_2 && steer_command > STEER_ZR) {
      steer_command = STEER_ZR;
      //Serial.println("Not sending steering1");
    } else if (steeringAngle <= PI_ON_2_3 && steeringAngle > PI && steer_command < STEER_ZR) {
      steer_command = STEER_ZR;
      //Serial.println("Not sending steering2");
    }
  
    // Force steer angle to zero
    // TODO
  
    //steer_command = pulse_to_position_command(rc_command[CH_STEER], steeringAngle,CH_STEER);
  
    motorDrivers.sendSteerCommand(steer_command);
    
  #endif // MOTORS_ON
  
  mass_right = load_right.get_units();
  mass_left = load_left.get_units();
  mass_rear = load_rear.get_units();

  data_line = "";
  #ifndef CSV_MODE
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
    data_line += "]";
    data_line += "[m:";
    data_line += mass_right;
    data_line += ",";
    data_line += mass_left;
    data_line += ",";
    data_line += mass_rear;
    data_line += "]\n";
  #else
    data_line += timer;
    data_line += ",";
    data_line += mpu_orien[VEC3_X] * RAD_TO_DEGREE;
    data_line += ",";
    data_line += mpu_orien[VEC3_Y] * RAD_TO_DEGREE;
    data_line += ",";
    data_line += mpu_orien[VEC3_Z] * RAD_TO_DEGREE;
    data_line += ",";
    data_line += mpu_acl[VEC3_X] * ACCL_G_PER_VAL_X * GRAVITY;
    data_line += ",";
    data_line += mpu_acl[VEC3_Y] * ACCL_G_PER_VAL_Y * GRAVITY;
    data_line += ",";
    data_line += mpu_acl[VEC3_Z] * ACCL_G_PER_VAL_Z * GRAVITY;
    data_line += ",";
    data_line += dist_travelled;
    data_line += ",";
    data_line += steeringAngle;
    data_line += ",";
    data_line += drive_command;
    data_line += ",";
    data_line += steer_command;
    data_line += ",";
    data_line += MPU.m_rawGyro[VEC3_X];
    data_line += ",";
    data_line += MPU.m_rawGyro[VEC3_Y];
    data_line += ",";
    data_line += MPU.m_rawGyro[VEC3_Z];
    data_line += ",";
    data_line += mass_right;
    data_line += ",";
    data_line += mass_left;
    data_line += ",";
    data_line += mass_rear;
    data_line += "\n";
  #endif
  
  
//============= SENDING ============//
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
  
  #ifdef RF_ON
    RF_serial->print(data_line);
  #endif //RF_ON
  
  #ifdef DEBUG_ON
    Serial.print(data_line); // Debugging
    Serial.println();
  #endif
  
  //delay(DELAY_T); // May not be necessary if there is more going on here
}

#ifdef WIFI_ON
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
#endif
