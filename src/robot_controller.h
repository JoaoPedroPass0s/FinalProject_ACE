#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

class robot_controller_t {
  public:
    robot_controller_t();

    float kpValues[3];
    float kiValues[3];
    float kdValues[3];
    float vValues[3];

    int mode;
    float previous_error; // For PID control
    float I ; // Integral term
    unsigned long interval; // Time interval for PID control

    float followLinePID(int ch1, int ch2, int ch3, int ch4, int ch5);
    std::pair<float,float> followEllipse(float a, float b, float theta);
    void changeMode();
};

#endif // MOTOR_CONTROLLER_H
