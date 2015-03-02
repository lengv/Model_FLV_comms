/*
  Control of motor drivers
*/
#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#define STEER_FR 1
#define STEER_ZR 64
#define STEER_FF 127

#define DRIVE_FR 128
#define DRIVE_ZR 192
#define DRIVE_FF 255

// Note extra bit is used for kill motors
#define DRIVE_RANGE 128
#define STEER_RANGE 127

/* Baudrates for motordriver, requires configuration on board
   1 2 3 4 5 6 - switch numbers
   u d u u u u - configuration with switch up or down
   Switch 6 d then slave select is activated
   Switch 3 d then lithium cutoff enabled (for severly dischared battery safety)
*/
//#define MOTOR_DRIVER_BAUDRATE 2400 // u d u u u u
#define MOTOR_DRIVER_BAUDRATE 9600 // u d u d u u
//#define MOTOR_DRIVER_BAUDRATE 19200 // u d u u d u
//#define MOTOR_DRIVER_BAUDRATE 38400 // u d u d d u

class Motor_Driver
{
  
  public:
    Motor_Driver(HardwareSerial *_drive, HardwareSerial *_steer){
      drive = _drive;
      steer = _steer;
    }
  
    void sendDriveCommand(unsigned char input){
      drive->write((input));
    }
    
    void sendSteerCommand(unsigned char input){
      steer->write((input));
    }
    
    void sendCommand(unsigned char drive_in, unsigned char steer_in){
      drive->write((drive_in));
      steer->write((steer_in));
    }
    
    void killAll()
    {
      drive->write(char(0));
      steer->write(char(0));
    }
    
    void killDrive(){drive->write(char(0));}
    void killSteer(){steer->write(char(0));}
    
  private:
    HardwareSerial *drive;
    HardwareSerial *steer;
    
};

#endif //MOTOR_DRIVER_H
