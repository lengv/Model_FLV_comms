/** Encapsulates Encoder.h and provides functionality for two encoder reads and distance measure
 */

#ifndef TRUCK_ENC_H

#define TRUCK_ENC_H 

#include <Encoder.h>

#define ENC_COUNTS_PER_REV 360 
#define ENC_PULSES_PER_REV 360*4 // 4X counting

#define WHEEL_DIAMETER 73 // mm
#define WHEEL_RADIUS WHEEL_DIAMETER/2.0

// Interrupt Pins for Quadrature Encoder [INT0 - INT5] -> [2,3,21,20,19,18]
#define ENC_QUAD_PIN_A 2 // Interrupt pin INT0
#define ENC_QUAD_PIN_B 3 // Interrupt pin INT1

// Absolute Encoder requiers Analog (Uses a pot)
#define ENC_ABS_PIN 4 // Any digital pin, avoiding interrupts
#define ENC_ABS_MAX 4096 // Max value

// Gear ratios to adjust count per rev
#define GEAR_RATIO_ENC 1.0
#define GEAR_RATIO_WHEEL 1.0

// Some Calculations on geometry of wheels
#define COUNTS_PER_REV ENC_PULSES_PER_REV*(GEAR_RATIO_ENC/GEAR_RATIO_WHEEL)

#define DIST_PER_COUNT (PI*WHEEL_DIAMETER)/COUNTS_PER_REV

// Geometry of steering
#define ANG_PER_VAL 2*PI/ENC_ABS_MAX

#define PI 3.1415926535897932384626433

class Truck_Enc
{
  
  public:
    Truck_Enc()
      :enc_quadrature(ENC_QUAD_PIN_A,ENC_QUAD_PIN_B){
		  // sets up quadrature pins
    }

    long readEncQuad(void){
      return enc_quadrature.read();
    }
    
    int readEncAbs(void){
      return digitalRead(ENC_ABS_PIN);
    }
    
    float readDist(void){
      return enc_quadrature.read()*DIST_PER_COUNT;
    }
    
    float readAngle(void){
      return enc_quadrature.read()*ANG_PER_VAL;
    }
    
    void serialWriteVals(HardwareSerial *serial){
      serial->print("[Enc:");
      serial->print(readDist());serial->print(",");
      serial->print(readAngle());serial->print("]");
    }
    
    void serialWriteRaw(HardwareSerial *serial){
      serial->print("[Enc:");
      serial->print(readEncQuad());serial->print(",");
      serial->print(readEncAbs());serial->print("]");
    }
  
  private:
    Encoder enc_quadrature;
    
};

void initEncPins(){
  pinMode(ENC_ABS_PIN,INPUT);
  // Note quadrature encoders pin modes are set up when class is constructed.
}

#endif
