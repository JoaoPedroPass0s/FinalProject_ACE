
#include <Arduino.h>
#include "robot_controller.h"

robot_controller_t::robot_controller_t()
{
  interval = 40;
  previous_error = 0; // For PID control
  I = 0; // Integral term
  mode = 0;
  kpValues[0] = 10.0;
  kpValues[1] = 30.0;
  kpValues[2] = 57.33;
  kiValues[0] = 0.0;
  kiValues[1] = 0.0;
  kiValues[2] = 2.60;
  kdValues[0] = 1.0;
  kdValues[1] = 1.5;
  kdValues[2] = 5.27;
  vValues[0] = 1.5;
  vValues[1] = 2.0;
  vValues[2] = 3.0;
}

int robot_controller_t::calculateNextMove(int x, int y, int currentDirection, int objects[GRID_ROWS][GRID_COLS], int finalX, int finalY) {
    // Define movement directions
    int dx[4] = {-1, 0, 1, 0};
    int dy[4] = {0, -1, 0, 1};
    int direction[4] = {LEFT, DOWN, RIGHT, UP};

    // Create a flood-fill grid
    int floodGrid[GRID_ROWS][GRID_COLS];
    for (int i = 0; i < GRID_ROWS; i++) {
        for (int j = 0; j < GRID_COLS; j++) {
            floodGrid[i][j] = (objects[i][j] == 0) ? -1 : INT_MAX; // -1 for empty cells, INT_MAX for obstacles
        }
    }

    // Flood-fill starting from the goal
    std::queue<std::pair<int, int>> q;
    q.push({finalY, finalX});
    floodGrid[finalY][finalX] = 0; // Goal has a distance of 0

    while (!q.empty()) {
        int currentY = q.front().first;
        int currentX = q.front().second;
        q.pop();

        for (int i = 0; i < 4; i++) {
            int nextX = currentX + dx[i];
            int nextY = currentY + dy[i];

            // Check bounds and if the cell hasn't been visited yet
            if (nextX >= 0 && nextX < GRID_COLS && nextY >= 0 && nextY < GRID_ROWS && floodGrid[nextY][nextX] == -1) {
                floodGrid[nextY][nextX] = floodGrid[currentY][currentX] + 1;
                q.push({nextY, nextX});
            }
        }
    }

    // Determine the next move based on floodGrid
    int bestDistance = INT_MAX;
    int nextDirection = -1;

    for (int i = 0; i < 4; i++) {
        int nextX = x + dx[i];
        int nextY = y + dy[i];

        // Check bounds and ensure the cell is not an obstacle
        if (nextX >= 0 && nextX < GRID_COLS && nextY >= 0 && nextY < GRID_ROWS && floodGrid[nextY][nextX] != INT_MAX) {
            if (floodGrid[nextY][nextX] < bestDistance || (floodGrid[nextY][nextX] == bestDistance && direction[i] == currentDirection)) {
                bestDistance = floodGrid[nextY][nextX];
                nextDirection = direction[i];
            }
        }
    }

    // If no valid direction found, return currentDirection to avoid turning back unnecessarily
    return (nextDirection == -1) ? currentDirection : nextDirection;
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

  if (total > 0) { // Normal Line Following
    line_position = (weights[0] * ch1 + weights[1] * ch2 + weights[2] * ch3 +
                      weights[3] * ch4 + weights[4] * ch5) / (float)total;
  }else if(previous_error < -0.5 || previous_error > 0.5){ // Sharp Turn
    line_position = previous_error;
  }else{
    line_position = 0; // No line detected -> Keep Going Straight
  }
  
  // PID control for angular velocity
  float error = line_position;

  float P = kpValues[mode] * error;
  I += kiValues[mode] * error * interval / 1000.0; // Integral term
  float D = kdValues[mode] * (error - previous_error) / (interval / 1000.0); // Derivative term
  float w_req = P + I + D;
  previous_error = error;

  return w_req;
}

void robot_controller_t::changeMode(){
  mode = (mode + 1) % 3;
}