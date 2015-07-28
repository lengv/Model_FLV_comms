/** Encapsulates Encoder.h and provides functionality for two encoder reads and distance measure
 */

#ifndef TRUCK_ENC_H

#define TRUCK_ENC_H 

#include <Encoder.h>

#define ENC_COUNTS_PER_REV 360 
#define ENC_PULSES_PER_REV 360*4.0 // 4X counting

#define WHEEL_DIAMETER 124 // mm
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

const float DIST_PER_COUNT = (PI*WHEEL_DIAMETER)/1440; // Note the division of 1/COUNTS_PER_REV does not work, so this will be done in it's place

// Geometry of steering
#define ANG_PER_VAL 2*PI/ENC_ABS_MAX

#define PI 3.1415926535897932384626433
#define PI_ON_2 PI/2
#define PI_ON_2_3 PI_ON_2*3

#define ANG_OFFSET -0.17-PI_ON_2

class Truck_Enc
{
  
  public:
    Truck_Enc()
      :enc_quadrature(ENC_QUAD_PIN_A,ENC_QUAD_PIN_B){
		  // sets up quadrature pins
      pinMode(ENC_ABS_PIN,INPUT);
    }
    
    void readEnc(void){
      encQuadVal = enc_quadrature.read();
      encAbsVal  = pulseIn(ENC_ABS_PIN,HIGH);
    }

    long getEncQuad(void){
      return encQuadVal;
    }
    
    int getEncAbs(void){
      return encAbsVal;
    }
    
    float getDist(void){
      return encQuadVal*DIST_PER_COUNT;
    }
    
    float getAngle(void){
      return encAbsVal*ANG_PER_VAL+ANG_OFFSET;
    }
    
    void serialWriteVals(HardwareSerial *serial){
      serial->print(F("[Enc:"));
      serial->print(getDist());serial->print(F(","));
      serial->print(getAngle());serial->print(F("]"));
    }
    
    void serialWriteRaw(HardwareSerial *serial){
      serial->print(F("[Enc:"));
      serial->print(encQuadVal);serial->print(F(","));
      serial->print(encAbsVal);serial->print(F("]"));
    }
    
    #ifdef WiFi_h
    void serialWriteVals(WiFiClient *client){
      client->print(F("[Enc:"));
      client->print(getDist());client->print(F(","));
      client->print(getAngle());client->print(F("]"));
    }
    
    void serialWriteRaw(WiFiClient *client){
      client->print(F("[Enc:"));
      client->print(encQuadVal);client->print(F(","));
      client->print(encAbsVal);client->print(F("]"));
    }
    #endif //WiFi_h
  
  private:
    Encoder enc_quadrature;
    int encQuadVal;
    float encAbsVal;
};

#endif
