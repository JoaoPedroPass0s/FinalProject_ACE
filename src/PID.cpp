
#include "Arduino.h"
#include "PID.h"

PID_t::PID_t()
{
  // Some typical values
  dt = 0.04;
  Kp = 1.3;
  Ki = 0.004;
  Kd = 0;
  Kf = 0.2;
  
  m_max = 5.8;
  m_min = -5.8;
}


float PID_t::calc(float new_w_ref, float new_w)
{
  float de;
  w = new_w;
  w_ref = new_w_ref;

  last_e = e;
  e = w_ref - w;
  
  // Integral and derivative of the error
  Se += e * dt;
  de = (e - last_e) / dt;
  
  // Calc PID output
  m = Kp * e + Ki * Se + Kd * de + Kf * w_ref;

  // Anti windup
  //if (m > m_max || m < m_min) {
  if ((m > m_max && e < 0) || (m < m_min && e > 0)) {
    // undo integration
    Se -= e * dt;
  }
  
  // Saturate the output
  if (m > m_max) {
    m = m_max;
  } else if (m < m_min) {
    m = m_min;
  }

  return m;
}
