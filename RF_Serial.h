/*
  RF serial wrapper
  Place holder for now in case anything more complicated is needed
*/

class RF_Serial
{
  
  public:
    RF_Serial(HardwareSerial *_serial){
      serial = _serial;
    }
/*
    void RFRead(){
      serial->read
    }
*/
    HardwareSerial* getRFSerial(){
      return serial;
    }
  
  private:
    HardwareSerial *serial;
    
};
