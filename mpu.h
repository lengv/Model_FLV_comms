#ifndef MPU_H

#define MPU_H 

#include "MPU9150Lib.h"

// ======================= SETUP ============================== //
//  MPU_UPDATE_RATE defines the rate (in Hz) at which the MPU updates the sensor data and DMP output

#define MPU_UPDATE_RATE  (20)

//  MAG_UPDATE_RATE defines the rate (in Hz) at which the MPU updates the magnetometer data
//  MAG_UPDATE_RATE should be less than or equal to the MPU_UPDATE_RATE

#define MAG_UPDATE_RATE  (10)

//  MPU_MAG_MIX defines the influence that the magnetometer has on the yaw output.
//  The magnetometer itself is quite noisy so some mixing with the gyro yaw can help
//  significantly. Some example values are defined below:

#define  MPU_MAG_MIX_GYRO_ONLY          0   // just use gyro yaw
#define  MPU_MAG_MIX_MAG_ONLY           1   // just use magnetometer and no gyro yaw
#define  MPU_MAG_MIX_GYRO_AND_MAG       10  // a good mix value 
#define  MPU_MAG_MIX_GYRO_AND_SOME_MAG  50  // mainly gyros with a bit of mag correction 

//  MPU_LPF_RATE is the low pas filter rate and can be between 5 and 188Hz

#define MPU_LPF_RATE   40


// ======================= FUNCTIONS ========================== //

// Following functions are adapted from the serial print from MPU9150Lib.h

// Writes the pose to a file

void writeAngle(HardwareSerial *input, float *vec)
{
  input->print('[');
  input->print("o:"); input->print(vec[VEC3_X] * RAD_TO_DEGREE);  
  input->print(","); input->print(vec[VEC3_Y] * RAD_TO_DEGREE);  
  input->print(","); input->print(vec[VEC3_Z] * RAD_TO_DEGREE);    
  input->print(']');
}

void writeAccl(HardwareSerial *input, short *vec)
{
  input->print('[');
  input->print("a:"); input->print(vec[VEC3_X]);  
  input->print(","); input->print(vec[VEC3_Y]);  
  input->print(","); input->print(vec[VEC3_Z]);    
  input->print(']');
}
#endif
