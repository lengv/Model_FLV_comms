#ifndef MPU_H

#define MPU_H 

  #include "MPU9150Lib.h"
  
  #define GRAVITY 9.80665
  
  // ======================= SETUP ============================== //
  //  MPU_UPDATE_RATE defines the rate (in Hz) at which the MPU updates the sensor data and DMP output
  
  #define MPU_UPDATE_RATE  (200)
  
  //  MAG_UPDATE_RATE defines the rate (in Hz) at which the MPU updates the magnetometer data
  //  MAG_UPDATE_RATE should be less than or equal to the MPU_UPDATE_RATE
  
  #define MAG_UPDATE_RATE  (100)
  
  //  MPU_MAG_MIX defines the influence that the magnetometer has on the yaw output.
  //  The magnetometer itself is quite noisy so some mixing with the gyro yaw can help
  //  significantly. Some example values are defined below:
  
  #define  MPU_MAG_MIX_GYRO_ONLY          0   // just use gyro yaw
  #define  MPU_MAG_MIX_MAG_ONLY           1   // just use magnetometer and no gyro yaw
  #define  MPU_MAG_MIX_GYRO_AND_MAG       10  // a good mix value 
  #define  MPU_MAG_MIX_GYRO_AND_SOME_MAG  50  // mainly gyros with a bit of mag correction 
  
  //  MPU_LPF_RATE is the low pas filter rate and can be between 5 and 188Hz
  
  #define MPU_LPF_RATE   50
  
  // Full scale range (fsr) - functions found in MotionDriver/inv_mpu.h
  #define MPU_ACCL_FSR_2   2 // +-  2 G
  #define MPU_ACCL_FSR_4   4 // +-  4 G
  #define MPU_ACCL_FSR_8   8 // +-  8 G
  #define MPU_ACCL_FSR_16 16 // +- 16 G
  
  #define MPU_ACCL_FSR MPU_ACCL_FSR_2
  
  // ======================= RANGING ============================ //
  #define ACCL_RANGE (MPU_ACCL_FSR*2)
  
  #define ACCL_MIN_X -8271
  #define ACCL_MAX_X  7962
  #define ACCL_MIN_Y -8108
  #define ACCL_MAX_Y  8089
  #define ACCL_MIN_Z -8278
  #define ACCL_MAX_Z  7831
  
  #define ACCL_RANGE_X (ACCL_MAX_X - ACCL_MIN_X)
  #define ACCL_RANGE_Y (ACCL_MAX_Y - ACCL_MIN_Y)
  #define ACCL_RANGE_Z (ACCL_MAX_Z - ACCL_MIN_Z)
  
  #define ACCL_G_PER_VAL_X (1.0*ACCL_RANGE/ACCL_RANGE_X)
  #define ACCL_G_PER_VAL_Y (1.0*ACCL_RANGE/ACCL_RANGE_Y)
  #define ACCL_G_PER_VAL_Z (1.0*ACCL_RANGE/ACCL_RANGE_Z)
  
  // ======================= FUNCTIONS ========================== //
  
  // Following functions are adapted from the serial print from MPU9150Lib.h
  
  // Writes the pose to a file
  
  void writeAngle(HardwareSerial *input, float *vec)
  {
    input->print(F("["));
    input->print(F("o:")); input->print(vec[VEC3_X] * RAD_TO_DEGREE);  
    input->print(F(",")); input->print(vec[VEC3_Y] * RAD_TO_DEGREE);  
    input->print(F(",")); input->print(vec[VEC3_Z] * RAD_TO_DEGREE);    
    input->print(F("]"));
  }
  
  void writeAccl(HardwareSerial *input, short *vec)
  {
    input->print(F("["));
    input->print(F("a:")); input->print(vec[VEC3_X]*ACCL_G_PER_VAL_X*GRAVITY);  
    input->print(F(",")); input->print(vec[VEC3_Y]*ACCL_G_PER_VAL_Y*GRAVITY);  
    input->print(F(",")); input->print(vec[VEC3_Z]*ACCL_G_PER_VAL_Z*GRAVITY);    
    input->print(F("]"));
  }
  
  void writeAcclVal(HardwareSerial *input, short *vec)
  {
    input->print(F("["));
    input->print(F("a:")); input->print(vec[VEC3_X]);  
    input->print(F(",")); input->print(vec[VEC3_Y]);  
    input->print(F(",")); input->print(vec[VEC3_Z]);    
    input->print(F("]"));
  }
  
  void writeAngleVal(HardwareSerial *input, float *vec)
  {
    input->print(F("["));
    input->print(F("o:")); input->print(vec[VEC3_X]);  
    input->print(F(",")); input->print(vec[VEC3_Y]);  
    input->print(F(",")); input->print(vec[VEC3_Z]);    
    input->print(F("]"));
  }
  
  #ifdef WiFi_h
    void writeAngle(WiFiClient *input, float *vec)
    {
      input->print(F("["));
      input->print(F("o:")); input->print(vec[VEC3_X] * RAD_TO_DEGREE);  
      input->print(F(",")); input->print(vec[VEC3_Y] * RAD_TO_DEGREE);  
      input->print(F(",")); input->print(vec[VEC3_Z] * RAD_TO_DEGREE);    
      input->print(F("]"));
    }
    
    void writeAccl(WiFiClient *input, short *vec)
    {
      input->print(F("["));
      input->print(F("a:")); input->print(vec[VEC3_X]*ACCL_G_PER_VAL_X*GRAVITY);  
      input->print(F(",")); input->print(vec[VEC3_Y]*ACCL_G_PER_VAL_Y*GRAVITY);  
      input->print(F(",")); input->print(vec[VEC3_Z]*ACCL_G_PER_VAL_Z*GRAVITY);    
      input->print(F("]"));
    }
    
    void writeAcclVal(WiFiClient *input, short *vec)
    {
      input->print(F("["));
      input->print(F("a:")); input->print(vec[VEC3_X]);  
      input->print(F(",")); input->print(vec[VEC3_Y]);  
      input->print(F(",")); input->print(vec[VEC3_Z]);    
      input->print(F("]"));
    }
    
    void writeAngleVal(WiFiClient *input, float *vec)
    {
      input->print(F("["));
      input->print(F("o:")); input->print(vec[VEC3_X]);  
      input->print(F(",")); input->print(vec[VEC3_Y]);  
      input->print(F(",")); input->print(vec[VEC3_Z]);    
      input->print(F("]"));
    }
  #endif //WiFi_h

#endif //MPU_H
