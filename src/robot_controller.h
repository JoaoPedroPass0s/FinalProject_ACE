#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

class robot_controller_t {
  public:
    robot_controller_t();

    float previous_error; // For PID control
    float I ; // Integral term
    float kp, ki, kd; // PID constants
    unsigned long interval; // Time interval for PID control
    float v; // Nominal velocity

    void setMotorPWM(int new_PWM, int pin_a, int pin_b);
    float followLinePID(int ch1, int ch2, int ch3, int ch4, int ch5);
    void followEllipse(float a, float b, float theta);
};

#endif // MOTOR_CONTROLLER_H
