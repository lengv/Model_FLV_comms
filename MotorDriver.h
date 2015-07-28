/*
  Control of motor drivers
*/
#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

//#define STEER_FR 16
//#define STEER_ZR 64
//#define STEER_FF 110
//
//#define DRIVE_FR 143
//#define DRIVE_ZR 192
//#define DRIVE_FF 239

//#define DRIVE_FR 1
//#define DRIVE_ZR 64
//#define DRIVE_FF 126
//
//#define STEER_FR 127
//#define STEER_ZR 192
//#define STEER_FF 254

#define STEER_FR 1
#define STEER_ZR 64
#define STEER_FF 126

#define DRIVE_FR 127
#define DRIVE_ZR 192
#define DRIVE_FF 255

// Note extra bit is used for kill motors
#define DRIVE_RANGE 127
#define STEER_RANGE 126

/* Baudrates for motordriver, requires configuration on board
   1 2 3 4 5 6 - switch numbers
   u d u u u u - configuration with switch up or down
   Switch 6 d then slave select is activated
   Switch 3 d then lithium cutoff enabled (for severly dischared battery safety)
*/
//#define MOTOR_DRIVER_BAUDRATE 2400 // u d u u u u
//#define MOTOR_DRIVER_BAUDRATE 9600 // u d u d u u
//#define MOTOR_DRIVER_BAUDRATE 19200 // u d u u d u
#define MOTOR_DRIVER_BAUDRATE 38400 // u d u d d u

class Motor_Driver
{
  
  public:
    Motor_Driver(HardwareSerial *_drive, HardwareSerial *_steer){
      driving = _drive;
      steer = _steer;
    }
  
    void sendDriveCommand(unsigned char input){
      driving->write((input));
    }
    
    void sendSteerCommand(unsigned char input){
      steer->write((input));
    }
    
    void sendCommand(unsigned char drive_in, unsigned char steer_in){
      driving->write((drive_in));
      steer->write((steer_in));
    }
    
    void killAll()
    {
      driving->write(char(0));
      steer->write(char(0));
    }
    
    void killDrive(){driving->write(char(0));}
    void killSteer(){steer->write(char(0));}
    
  private:
    HardwareSerial *driving;
    HardwareSerial *steer;
    
};

#endif //MOTOR_DRIVER_H
