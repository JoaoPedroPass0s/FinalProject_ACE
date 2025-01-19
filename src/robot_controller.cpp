
#include <Arduino.h>
#include "robot_controller.h"

robot_controller_t::robot_controller_t()
{
  interval = 40;
  previous_error = 0; // For PID control
  I = 0; // Integral term
  kp = 55.0;
  ki = 0.1;
  kd = 2.5; 
  v = 3;
}

std::pair<float,float> robot_controller_t::followEllipse(float a, float b, float theta)
{
  // Calculate the robot's velocity in x and y directions
  float vx = -a * sin(theta);
  float vy = b * cos(theta);

  float maxVel = sqrt((a * a) + (b * b));
  
  // Calculate the linear velocity (magnitude of velocity vector)
  float v = sqrt(vx * vx + vy * vy);

  // Calculate the angular velocity (how fast the robot is turning)
  float w = (vy * a - vx * b) / (a * b); // This is the angular velocity around the object
  
    // Calculate velocities for the left and right wheels
  float v1 = v - (w * 0.105 / 2);  // Left wheel velocity
  float v2 = v + (w * 0.105 / 2);  // Right wheel velocity
  
  // Convert wheel velocities to PWM values
  float PWM_1 = (v1 * 255) / maxVel;  // Normalize to PWM range (0-255)
  float PWM_2 = (v2 * 255) / maxVel;  // Normalize to PWM range (0-255)

  // Ensure PWM values are within the allowed range
  PWM_1 = constrain(PWM_1, 0, 255);
  PWM_2 = constrain(PWM_2, 0, 255);

  return std::make_pair(PWM_1, PWM_2);
}

float robot_controller_t::followLinePID(int ch1, int ch2, int ch3, int ch4, int ch5){
  
  // Calculate line position
  int weights[5] = {-2, -1, 0, 1, 2};
  float line_position = 0;
  int total = 5 - (ch1 + ch2 + ch3 + ch4 + ch5);

  if (total > 0) {
    line_position = (weights[0] * ch1 + weights[1] * ch2 + weights[2] * ch3 +
                      weights[3] * ch4 + weights[4] * ch5) / (float)total;
  }else{
    line_position = previous_error;
  }
  
  // PID control for angular velocity
  float error = line_position;

  float P = kp * error;
  I += ki * error * interval / 1000.0; // Integral term
  float D = kd * (error - previous_error) / (interval / 1000.0); // Derivative term
  float w_req = P + I + D;
  previous_error = error;

  return w_req;
}