/*
  Control of motor drivers
*/

class Motor_Driver
{
  
  public:
    Motor_Driver(HardwareSerial *_drive, HardwareSerial *_steer){
      drive = _drive;
      steer = _steer;
    }
  
    void sendDriveCommand(char input){
      drive->write(input);
    }
    
    void sendSteerCommand(char input){
      steer->write(input);
    }
    
    void sendCommand(char drive_in, char steer_in){
      drive->write(drive_in);
      steer->write(steer_in);
    }
  
  private:
    HardwareSerial *drive;
    HardwareSerial *steer;
    
};
