/*
 Radio Control Library
 Defines 3 Pins for 3 channel RC
 Print function associated
*/

#define RC_PIN_CH1 34
#define RC_PIN_CH2 6
#define RC_PIN_CH3 7

// 
int rc_read_pulse(int pin)
{
  return pulseIn(pin,HIGH,25000);
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

#endif
