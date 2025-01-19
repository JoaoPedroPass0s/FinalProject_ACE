
#ifndef PID_H
#define PID_H

#include "Arduino.h"


class PID_t
{
  public:
    float dt;
    float Ki, Kp, Kd, Kf;
    float w, w_ref;
    float e, last_e, Se;
    float m, m_max, m_min;
    uint8_t active;

    PID_t();
    
    float calc(float new_w_ref, float new_w);
};

#endif // PID_H
