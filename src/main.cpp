#include <Arduino.h>

#define ENCA1 18   // Encoder A for motor 1
#define ENCB1 17   // Encoder B for motor 1
#define ENCB2 16   // Encoder A for motor 2
#define ENCA2 15   // Encoder B for motor 2
#define D0 13      // Motor 1 IN1
#define D1 14      // Motor 1 IN2
#define D2 11      // Motor 2 IN1
#define D3 12      // Motor 2 IN2
#define CHANNEL_1 0 // GPIO0
#define CHANNEL_2 1 // GPIO1
#define CHANNEL_3 2 // GPIO2
#define CHANNEL_4 3 // GPIO3
#define CHANNEL_5 4 // GPIO4

#define BUTTON_START 10  // Pin for start button

const int BASE_SPEED = 100;  // Base speed for the motors (range: 0-255 for PWM)
const float Kp = 20.0;       // Proportional gain

volatile int posi1 = 0; // Encoder position for motor 1
volatile int posi2 = 0; // Encoder position for motor 2
long prevT = 0;
float eprev1 = 0, eprev2 = 0;
float eintegral1 = 0, eintegral2 = 0;

bool running = false; // Flag to indicate if the robot is moving

// Define waypoints (target positions)
int waypoints[] = {0, 500, 0, 500, 0}; // Add more points as needed
int numWaypoints = sizeof(waypoints) / sizeof(waypoints[0]); // Number of waypoints
int currentWaypoint = 0; // Index to track current target position

void setMotor(int dir, int pwmVal, int in1, int in2);
void readEncoder1();
void readEncoder2();

void setup() {
  Serial.begin(9600);
  pinMode(ENCA1, INPUT);
  pinMode(ENCB1, INPUT);
  pinMode(ENCA2, INPUT);
  pinMode(ENCB2, INPUT);

  // Set sensor pins as inputs
  pinMode(CHANNEL_1, INPUT);
  pinMode(CHANNEL_2, INPUT);
  pinMode(CHANNEL_3, INPUT);
  pinMode(CHANNEL_4, INPUT);
  pinMode(CHANNEL_5, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA1), readEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA2), readEncoder2, RISING);

  pinMode(D0, OUTPUT);
  pinMode(D1, OUTPUT);
  pinMode(D2, OUTPUT);
  pinMode(D3, OUTPUT);

  Serial.println("Starting path following...");

  setMotor(0, 0, D0, D1);
  setMotor(0, 0, D2, D3);   
}

void loop() {

  if(digitalRead(BUTTON_START) == LOW){
    running = !running;
  }

  // Read digital values from each channel
  int s1 = digitalRead(CHANNEL_1);
  int s2 = digitalRead(CHANNEL_2);
  int s3 = digitalRead(CHANNEL_3);
  int s4 = digitalRead(CHANNEL_4);
  int s5 = digitalRead(CHANNEL_5);

    // Calculate line position (weighted average)
  int position = (s1 * -2) + (s2 * -1) + (s3 * 0) + (s4 * 1) + (s5 * 2);

  // Calculate error from the center
  int error = position;

  // Compute correction using proportional control
  int correction = Kp * error;

  // Calculate motor speeds
  int leftMotorSpeed = constrain(BASE_SPEED - correction, 0, 255);
  int rightMotorSpeed = constrain(BASE_SPEED + correction, 0, 255);

  if(running){
    setMotor(1, leftMotorSpeed, D0, D1);
    setMotor(1, rightMotorSpeed, D2, D3);    
  }
}

void setMotor(int dir, int pwmVal, int in1, int in2) {
  if (dir == 1) {
    analogWrite(in1, pwmVal);
    analogWrite(in2, 0);
  } else if (dir == -1) {
    analogWrite(in1, 0);
    analogWrite(in2, pwmVal);
  } else {
    analogWrite(in1, 0);
    analogWrite(in2, 0);
  }
}

void readEncoder1() {
  int b = digitalRead(ENCB1);
  if (b > 0) {
    posi1++;
  } else {
    posi1--;
  }
}

void readEncoder2() {
  int b = digitalRead(ENCB2);
  if (b > 0) {
    posi2++;
  } else {
    posi2--;
  }
}
