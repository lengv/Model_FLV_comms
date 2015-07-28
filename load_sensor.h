#include <HX711.h>

void init_load_sensors(HX711* right, HX711* left, HX711* rear){
  // Calibrated scale factor, assuming linear response (note hysteris is considered small enough to ignore)
  right->set_scale(30.4489855635185);
  left->set_scale(30.7872367359259);
  rear->set_scale(31.3596887682408);
  
  // Tared weight of only springs
  right->set_offset(7560334);
  //left->set_offset(7719497);
  left->set_offset(7767533);
  rear->set_offset(8344577);
}
