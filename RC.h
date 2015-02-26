/*
 Radio Control Library
 Defines 3 Pins for 3 channel RC
 Print function associated
 Requires MotorDriver.h -> levels inputs in accordance to range required by motor drivers
*/
#ifndef RC_H
#define RC_H

#include "MotorDriver.h"

// Only require digital pins
#define RC_PIN_CH1 34
#define RC_PIN_CH2 35
#define RC_PIN_CH3 36

// Max min values of RC signal - Not #def to allow online calibration (if necessary)
int ch_min[3] = {1100, 1100, 1100};
int ch_max[3] = {1500, 1500, 1500};
int ch_range[3] = {400, 400, 400};

int command_min[3] = {0, DRIVE_FR, STEER_FR};
int command_max[3] = {0, DRIVE_FF, STEER_FF};
int command_range[3] = {0, DRIVE_RANGE, STEER_RANGE};

#define CH_DRIVE = 2;
#define CH_STEER = 3;

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
char pulse_to_command(int pulseVal, int channel)
{
  float commandVal;
  char roundedCommandVal;
  
  commandVal = (float(pulseVal-ch_min[channel-1]) / ch_range[channel-1])*command_min[channel-1] + command_range[channel-1];
  
  roundedCommandVal = char(round(commandVal));
  return roundedCommandVal;
}

#ifdef DEBUG_ON
// DEBUG - print all 3 channels to hardware serial 
void print_channel(HardwareSerial *serial, int ch1, int ch2, int ch3)
{
  serial->print("[RC:");
  serial->print(ch1);serial->print(",");
  serial->print(ch2);serial->print(",");
  serial->print(ch3);serial->print("]");
}

#endif //DEBUG_ON

#endif //RC_H
