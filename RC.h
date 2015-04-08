/*
 Radio Control Library
 Defines 3 Pins for 3 channel RC
 Print function associated
 Requires MotorDriver.h -> levels inputs in accordance to range required by motor drivers
*/
#ifndef RC_H
#define RC_H

//#include <WiFi.h>
#include "MotorDriver.h"

// Only require digital pins
#define RC_PIN_CH1 34
#define RC_PIN_CH2 35
#define RC_PIN_CH3 36

#define RC_STEER_PIN RC_PIN_CH2
#define RC_DRIVE_PIN RC_PIN_CH3

// Max min values of RC signal - Not #def to allow online calibration (if necessary)
int ch_min[3] = {1100, 1121, 1112};
int ch_max[3] = {1900, 1778, 1786};
int ch_range[3] = {800, 657, 674};

//int command_min[3] = {0, DRIVE_FR, STEER_FR};
//int command_max[3] = {0, DRIVE_FF, STEER_FF};
//int command_range[3] = {0, DRIVE_RANGE, STEER_RANGE};

int command_min[3] = {0, STEER_FR, DRIVE_FR};
int command_max[3] = {0, STEER_FF, DRIVE_FF};
int command_range[3] = {0, STEER_RANGE, DRIVE_RANGE};

#define CH_DRIVE 3-1
#define CH_STEER 2-1

#define CUTOFF PI/3

// 
int rc_read_pulse(int pin)
{
  return pulseIn(pin,HIGH,25000); // Reads the length of pulse. Works well for 10ms to 3mins. Returns length of pulse
}

/** Brief - Converts pulses to commands given specified ranges 
 **
 ** pulseVal - RC pulse channel values
 ** channel  - use CH_DRIVE/CH_STEER
 ** returns command value rounded as char
 **/
unsigned char pulse_to_command(int pulseVal, int channel)
{
  if(pulseVal == 0){
    return 0;
  }
  float commandVal;
  unsigned char roundedCommandVal;
  
  commandVal = ((float(pulseVal-ch_min[channel]) / ch_range[channel])*(float (command_range[channel]))) + command_min[channel];
  
  if (commandVal < command_min[channel]){
    commandVal = command_min[channel];
  } else if (commandVal > command_max[channel]){
    commandVal = command_max[channel];
  }
  
  roundedCommandVal = char(round(commandVal));
  return roundedCommandVal;
}

unsigned char pulse_to_position_command(int pulseVal, float steering_angle, int channel)
{
  if(pulseVal == 0){
    return 0;
  }
  float commandAng;
  uint8_t commandVal;
  
  commandAng = ((float(pulseVal-ch_min[channel]) / ch_range[channel])*(float (PI))) - PI_ON_2;
  Serial.print(F("CommandAng:"));Serial.println(commandAng);
  if (commandAng < -PI_ON_2){
    commandAng = -PI_ON_2;
  } else if (commandAng > PI_ON_2){
    commandAng = PI_ON_2;
  }
  
//  if(steering_angle > PI){
//    steering_angle = 2*PI - steering_angle;
//  }
  
  float diff = commandAng - steering_angle;
  Serial.print(F("SA:"));Serial.println(steering_angle);
  Serial.print(F("Diff:"));Serial.println(diff);

  if(diff < CUTOFF && diff > 0){
    commandVal = diff/(CUTOFF)*(STEER_FF-STEER_ZR) + STEER_ZR;
    Serial.print("1");
    
  } else if(diff < 0 && diff > -CUTOFF){
    commandVal = STEER_ZR + diff/(CUTOFF)*(STEER_ZR-STEER_FR)+STEER_FR;
    Serial.print("2");
  } else if(diff <= -CUTOFF){
    commandVal = STEER_FR;
    Serial.print("3");
  } else if(diff >= CUTOFF){
    commandVal = STEER_FF;
    Serial.print("4");
  } else{
    commandVal = STEER_ZR;
    Serial.print("5");
  }
  
  return commandVal;
}

void print_channel(HardwareSerial *serial, int ch1, int ch2, int ch3)
{
  serial->print(F("[RC:"));
  serial->print(ch1);serial->print(F(","));
  serial->print(ch2);serial->print(F(","));
  serial->print(ch3);serial->print(F("]"));
}

#ifdef WiFi_h
void print_channel(WiFiClient *client, int ch1, int ch2, int ch3)
{
  client->print(F("[RC:"));
  client->print(ch1);client->print(F(","));
  client->print(ch2);client->print(F(","));
  client->print(ch3);client->print(F("]"));
}
#endif //WiFi_h

#endif //RC_H
