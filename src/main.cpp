#include <Arduino.h>
#include <WiFi.h>

#include "pico/cyw43_arch.h"

#include <Wire.h>
#include <VL53L0X.h>

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

VL53L0X tof;

// Time variables
unsigned long currentMicros, previousMicros, interval;

//Distance sensor variables
float sensorDist, prev_sensorDist;

// Line following Sensor variables
int ch1, ch2, ch3, ch4, ch5;

// Encoder variables
volatile int posi1 = 0; // Encoder position for motor 1
volatile int posi2 = 0; // Encoder position for motor 2

const int BASE_SPEED = 100;  // Base speed for the motors (range: 0-255 for PWM)
const float Kp = 20.0;       // Proportional gain

long prevT = 0;
float eprev1 = 0, eprev2 = 0;
float eintegral1 = 0, eintegral2 = 0;

bool running = true; // Flag to indicate if the robot is moving

// Define waypoints (target positions)
int waypoints[] = {0, 500, 0, 500, 0}; // Add more points as needed
int numWaypoints = sizeof(waypoints) / sizeof(waypoints[0]); // Number of waypoints
int currentWaypoint = 0; // Index to track current target position

void setMotor(int dir, int pwmVal, int in1, int in2);
void readEncoder1();
void readEncoder2();
void controlCar();
void displayInfo();

void setup() {
  Serial.begin(9600);

  interval = 400000;

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

  Wire.setSDA(20);
  Wire.setSCL(21);

  Wire.begin();

  tof.setTimeout(500);
  while (!tof.init()) {
    Serial.println(F("Failed to detect and initialize VL53L0X!"));
    delay(100);
  }  

  // Reduce timing budget to 20 ms (default is about 33 ms)
  //tof.setMeasurementTimingBudget(20000);

  // Start new distance measure
  tof.startReadRangeMillimeters();  
}

void loop() {
  currentMicros = micros();

  ch1 = digitalRead(CHANNEL_1);
  ch2 = digitalRead(CHANNEL_2);
  ch3 = digitalRead(CHANNEL_3);
  ch4 = digitalRead(CHANNEL_4);
  ch5 = digitalRead(CHANNEL_5);

  if (currentMicros - previousMicros >= interval) {
    previousMicros = currentMicros;

    // Read sensor distance
    if (tof.readRangeAvailable()) {
      prev_sensorDist = sensorDist;
      sensorDist = tof.readRangeMillimeters() * 1e-3;
    }

    displayInfo();
  }
}

void displayInfo() {
    Serial.print("encoder1: ");
    Serial.print(posi1);
    Serial.print(", encoder2: ");
    Serial.println(posi2);

    Serial.print("Channels: ");
    Serial.print(ch1);
    Serial.print(ch2);
    Serial.print(ch3);
    Serial.print(ch4);
    Serial.print(ch5);

    Serial.print(" Dist: ");
    Serial.print(sensorDist, 3);
    Serial.println();
}

void controlCar() {

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
