
#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <math.h>
#include "PID.h"


typedef enum { 
  cm_pwm,
  cm_pid
} control_mode_t;

class robot_t {
  public:
  int enc1, enc2;
  int Senc1, Senc2;
  float w1e, w2e;
  float v1e, v2e;
  float ve, we;
  float ds, dtheta;
  float rel_s, rel_theta;
  float xe, ye, thetae;
  
  float dt;
  float v, w;
  float v_req, w_req;
  float dv_max, dw_max;
  
  float wheel_radius, wheel_dist;
  
  float v1ref, v2ref;
  float w1ref, w2ref;
  float u1, u2;
  int PWM_1, PWM_2;
  int PWM_1_req, PWM_2_req;
  control_mode_t control_mode;
  
  PID_t PID1, PID2;
  float battery_voltage;
  
  robot_t();

  void odometry(void);
  void setRobotVW(float Vnom, float Wnom);
  void tunePID(float kp, float ki, float kd);

  void accelerationLimit(void);
  void VWToMotorsVoltage(void);

  void updateVoltage(void);
};


#endif // ROBOT_H
