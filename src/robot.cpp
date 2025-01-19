

#include <Arduino.h>
#include "robot.h"

robot_t::robot_t()
{
  wheel_dist = 0.105;
  wheel_radius = 0.0689 / 2;
  dv_max = 5;
  dw_max = 20;
  dt = 0.04;
}

void robot_t::odometry(void)
{
  // Estimate wheels speed using the encoders
  w1e = enc1 * TWO_PI / (2.0 * 1920.0 * dt);
  w2e = enc2 * TWO_PI / (2.0 * 1920.0 * dt);

  v1e = w1e * wheel_radius;
  v2e = w2e * wheel_radius;

  // Estimate robot speed
  ve = (v1e + v2e) / 2.0;
  we = (v1e - v2e) / wheel_dist;
  
  // Estimate the distance and the turn angle
  ds = ve * dt;
  dtheta = we * dt;

  // Estimate pose
  xe += ds * cos(thetae + dtheta/2);
  ye += ds * sin(thetae + dtheta/2);
  thetae = thetae + dtheta;

  // Relative displacement
  rel_s += ds;
  rel_theta += dtheta;
}

void robot_t::setRobotVW(float Vnom, float Wnom)
{
  v_req = Vnom;
  w_req = Wnom;
}


void robot_t::tunePID(float kp, float ki, float kd)
{
  PID1.Kp = kp;
  PID1.Ki = ki;
  PID1.Kd = kd;
  PID2.Kp = kp;
  PID2.Ki = ki;
  PID2.Kd = kd; 
}

void robot_t::accelerationLimit(void)
{
  float dv = v_req - v;
  dv = constrain(dv, -dv_max, dv_max);
  v += dv;

  float dw = w_req - w;
  dw = constrain(dw, -dw_max, dw_max);
  w += dw;
}


void robot_t::VWToMotorsVoltage(void)
{
  v1ref = v + w * wheel_dist / 2;
  v2ref = v - w * wheel_dist / 2; 

  w1ref = v1ref * wheel_radius;
  w2ref = v2ref * wheel_radius;

  if (control_mode == cm_pwm) {
    PWM_1 = PWM_1_req;  
    PWM_2 = PWM_2_req;  

  } else if (control_mode == cm_pid) {
    u1 = 0;
    u2 = 0;      

    if (v1ref != 0) u1 = PID1.calc(v1ref, v1e);
    else PID1.Se = 0;

    if (v2ref != 0) u2 = PID2.calc(v2ref, v2e);
    else PID2.Se = 0;

    PWM_1 = u1 / battery_voltage * 255;
    PWM_2 = u2 / battery_voltage * 255;
  }
}

void robot_t::updateVoltage() {
  int value = analogRead(A2);
  float v_out = (value * 3.3f) / 4095.0f;
  float v_in = v_out * (330000.0f + 100000.0f) / 100000.0f;
  battery_voltage = v_in;
  //battery_voltage = 7.2;
}