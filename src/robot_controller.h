#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#define GRID_ROWS 5
#define GRID_COLS 6

// Direction
enum {
  LEFT = 0,
  UP,
  RIGHT,
  DOWN
};

class robot_controller_t {
  public:
    robot_controller_t();

    float kpValues[4];
    float kiValues[4];
    float kdValues[4];
    float vValues[4];

    int mode;
    float previous_error; // For PID control
    float I ; // Integral term
    unsigned long interval; // Time interval for PID control

    int calculateNextMove(int x, int y, int currentDirection, int objects[GRID_ROWS][GRID_COLS], int finalX, int finalY);
    float followLinePID(int ch1, int ch2, int ch3, int ch4, int ch5);
    float followEllipse(float a, float b, float theta, float v);
    void changeMode();
};

#endif // MOTOR_CONTROLLER_H
