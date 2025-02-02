#include <Arduino.h>

#include "pico/cyw43_arch.h"
#include <WiFi.h>
#include <Wire.h>
#include <VL53L0X.h>
#include "robot.h"
#include "robot_controller.h"
#include <RPi_Pico_TimerInterrupt.h>
#include <Bounce2.h>

#define CYW43_WL_GPIO_LED_PIN 0
#define TEST_PIN 27
#define ENCA1 18   // Encoder A for motor 1
#define ENCB1 17   // Encoder B for motor 1
#define ENCB2 16   // Encoder A for motor 2
#define ENCA2 15   // Encoder B for motor 2
#define D0 13      // Motor 1 IN1
#define D1 14      // Motor 1 IN2
#define D2 11      // Motor 2 IN1
#define D3 12      // Motor 2 IN2
#define BUTTON1 5
#define BUTTON2 6
#define BUTTON3 7

// Multiplexer control pins
#define S0 10  
#define S1 9  
#define S2 8  

// ADC pin connected to MUX output
#define MUX_ADC_PIN 27  // GPIO27 (ADC1)

// Number of sensors
#define NUM_SENSORS 5

// Analog threshold to determine black/white (adjust based on your sensors)
#define THRESHOLD 2500  // Experiment and adjust

#define digitalWriteFast(pin, val)  (val ? sio_hw->gpio_set = (1 << pin) : sio_hw->gpio_clr = (1 << pin))
#define digitalReadFast(pin)        (((1 << pin) & sio_hw->gpio_in) >> pin)

#define pinIsHigh(pin, pins)        (((1 << pin) & pins) >> pin)

// fsm states LF
enum {
  sm1_lineFollowing = 0,
  sm1_stop,
  sm1_turn1,
  sm1_adjust1,
  sm1_move1,
  sm1_turn2,
  sm1_move2,
};

//fsm states GMF
enum{
  sm2_scan = 0,
  sm2_lineFollowing,
  sm2_turnRight,
  sm2_turnLeft,
  sm2_turnBack,
  sm2_goBack,
  sm2_stop,
};

VL53L0X tof;

robot_t robot;

robot_controller_t robot_controller;

Bounce debouncerButton1 = Bounce();
Bounce debouncerButton2 = Bounce();
Bounce debouncerButton3 = Bounce();

// Web server on port 80
WiFiServer server(80);

// Init RPI_PICO_Timer
RPI_PICO_Timer ITimer1(1);

typedef struct {
  int state, new_state;

  // tes - time entering state
  // tis - time in state
  // tup - time since last update
  unsigned long tes, tis, tup;
} fsm_t;

fsm_t fsm_LF; // Line Follower

fsm_t fsm_GMS; // Grid Maze Solver

int mode;

int LED_state; // LED state

bool itsRunning = false;

// WiFi credentials
const char* ssid = "HONOR View20";       // Replace with your WiFi SSID
const char* password = "robottest"; // Replace with your WiFi password

WiFiClient currentClient = server.available();

// Time variables
unsigned long currentMicros;

// Button variables
uint8_t button1;
uint8_t button2;
uint8_t button3;

unsigned long pressStartTime = 0;

//Distance sensor variables
float sensorDist = 0 ;

const int consistentReadings = 3;
static int objectCount = 0;

// Line following Sensor variables
int sensorValues[NUM_SENSORS];
int binaryValues[NUM_SENSORS];

static int currentSensor = 0;
static unsigned long lastReadTime = 0;
const unsigned long sensorReadInterval = 2;  // Adjust based on response time

static unsigned long lastTOFSensorRead = 0;
static unsigned long lastPrintTime = 0;

int enc1, enc2;

volatile int encoder1_pos = 0;
volatile int encoder2_pos = 0;

int encoder1_state, encoder2_state;
// This table implements the counting event for every transition of the encoder state machine
int encoder_table[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0}; 

volatile int count;
int act_count;

unsigned long interval;

int x = 0,y = 0;

int currentDirection = UP;

void read_encoders();
bool timer_handler(struct repeating_timer *t);
void displayInfo();
void controlRobotLFStm();
void controlRobotGMSStm();
int calculateTurnState(int currentDirection, int nextDirection);
void set_state(fsm_t& fsm, int new_state);
void setRobotVW(float Vnom, float Wnom);
void setMotorPWM(int new_PWM, int pin_a, int pin_b);
void receiveData();
void changeMode();
void initWifi();
void selectChannel(int channel);
int readSensor(int channel);

void setup() {
  Serial.begin(9600);

  analogReadResolution(12); // Set ADC resolution to 12 bits

  pinMode(ENCA1, INPUT);
  pinMode(ENCB1, INPUT);
  pinMode(ENCA2, INPUT);
  pinMode(ENCB2, INPUT);

  pinMode(D0, OUTPUT);
  pinMode(D1, OUTPUT);
  pinMode(D2, OUTPUT);
  pinMode(D3, OUTPUT);

  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(BUTTON3, INPUT_PULLUP);

  // Set MUX select pins as outputs
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);

  debouncerButton1.attach(BUTTON1);
  debouncerButton1.interval(50); // 50 ms debounce delay
  debouncerButton2.attach(BUTTON2);
  debouncerButton2.interval(50); // 50 ms debounce delay
  debouncerButton3.attach(BUTTON3);
  debouncerButton3.interval(50); // 50 ms debounce delay  

  if (ITimer1.attachInterrupt(40000, timer_handler))
    Serial.println("Starting ITimer OK, millis() = " + String(millis()));
  else
    Serial.println("Can't set ITimer. Select another freq. or timer");

  Wire.setSDA(20);
  Wire.setSCL(21);

  Wire.begin();

  tof.setTimeout(500);
  while (!tof.init()) {
    Serial.println(F("Failed to detect and initialize VL53L0X!"));
    delay(100);
  }  

  // Reduce timing budget to 20 ms (default is about 33 ms)
  tof.setMeasurementTimingBudget(20000);

  // Start new distance measure
  tof.startReadRangeMillimeters();  

  interval = 40;             // In miliseconds
  robot.dt = 1e-3 * interval; // In seconds
  robot.PID1.dt = robot.dt;
  robot.PID2.dt = robot.dt;
  robot.control_mode = cm_pid;

  set_state(fsm_LF, sm1_lineFollowing); 
  set_state(fsm_GMS, sm2_scan);

  mode = 0; // Line following mode

  Serial.println("Starting path following...");

}

void loop() {

  if(!currentClient){
    currentClient = server.available();
    if (currentClient) {
      Serial.println("Client connected.");
    }
  }

  debouncerButton1.update();
  debouncerButton2.update();
  debouncerButton3.update();
  button1 = debouncerButton1.fell();
  button2 = debouncerButton2.fell();
  button3 = debouncerButton3.fell();

  if (millis() - lastReadTime >= sensorReadInterval) {
    lastReadTime = millis();  // Update last read time

    // Read current sensor
    sensorValues[currentSensor] = readSensor(currentSensor);
    binaryValues[currentSensor] = sensorValues[currentSensor] < THRESHOLD ? 0 : 1;

    // Move to the next sensor
    currentSensor = (currentSensor + 1) % NUM_SENSORS;
  }
  
  currentMicros = millis();

  if(button1){
    itsRunning = !itsRunning;
  }
  if(button2){
    robot_controller.changeMode(); // Change PID Mode 
  }
  if(button3){
    changeMode();
  }

    // Check if the button is pressed for 3 seconds
  if (!debouncerButton1.read()) {
      if (pressStartTime == 0) pressStartTime = millis();
      if (millis() - pressStartTime > 3000){
        initWifi(); // Reconnect to WiFi
        pressStartTime = 0;
      }  
  } else {
      pressStartTime = 0;
  }

  // Call receiveData
  if (currentClient) {
    receiveData();
  }

  displayInfo();

  if(currentMicros - lastTOFSensorRead >= interval){
    lastTOFSensorRead = currentMicros;
    if (tof.readRangeAvailable()) {
        float currentReading = tof.readRangeMillimeters() * 1e-3;

        if (currentReading < 0.1) {
            objectCount++;
            if (objectCount >= consistentReadings) {
                sensorDist = currentReading; // Confirm object
            }
        } else {
            objectCount = 0; // Reset if out of range
            sensorDist = currentReading;
        }
    }
    tof.startReadRangeMillimeters();
  }

  robot.updateVoltage();

  if(robot.battery_voltage < 6.0 || !itsRunning){
    robot.PWM_1 = 0;
    robot.PWM_2 = 0;
    setMotorPWM(robot.PWM_1, D1, D0);
    setMotorPWM(robot.PWM_2, D3, D2);
    if(currentMicros % 500 == 0){
      LED_state = !LED_state;
      cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, LED_state);
    }
  }else{
    if(mode == 0){
      controlRobotLFStm();
    }else{
      controlRobotGMSStm();
    }
  }
}

void initWifi(){
  // Initialize WiFi
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected.");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Start the server
  server.begin();
}

// Function to select a channel on the MUX
void selectChannel(int channel) {
  digitalWrite(S0, channel & 1);
  digitalWrite(S1, (channel >> 1) & 1);
  digitalWrite(S2, (channel >> 2) & 1);
}

// Function to read analog values from sensors through the multiplexer
int readSensor(int channel) {
  selectChannel(channel);
  return analogRead(MUX_ADC_PIN);  // Read ADC value
}

void changeMode(){
  if(mode == 0){
    set_state(fsm_GMS, sm2_scan);
    mode = 1;
  }else{
    set_state(fsm_LF, sm1_lineFollowing);
    mode = 0;
  }
}

// Set new state
void set_state(fsm_t& fsm, int new_state)
{
  if (fsm.state != new_state) {
    fsm.state = new_state;
    fsm.tes = millis();
    fsm.tis = 0;
    fsm.tup = millis();
  }
}

float turnAngle = 0;
float a = 0;

void controlRobotLFStm() {
  fsm_LF.tis = millis();

  // State transitions
  if ((fsm_LF.state == sm1_lineFollowing || fsm_LF.state == sm1_move1 || fsm_LF.state == sm1_turn2) && sensorDist <= 0.1) {
    fsm_LF.new_state = sm1_stop; // Start turning
  }else if(fsm_LF.state == sm1_stop && fsm_LF.tis - fsm_LF.tes > 100){
    a = sensorDist + 0.005;
    robot.rel_theta = 0;
    fsm_LF.new_state = sm1_turn1; // Turn in place
  } else if (fsm_LF.state == sm1_turn1 && sensorDist > 0.20) {
    turnAngle = robot.rel_theta;
    fsm_LF.new_state = sm1_adjust1; // Rotate to adjust state
    robot.rel_theta = 0;
  } else if (fsm_LF.state == sm1_adjust1 && robot.rel_theta <= -0.8) {
    fsm_LF.new_state = sm1_move1; // Rotate to move state
    turnAngle += robot.rel_theta;
    Serial.println(turnAngle);
    robot.rel_theta = 0;
  } else if (fsm_LF.state == sm1_move1 && ((abs(robot.rel_theta) - abs((turnAngle * 2) - 0.1)) >= 0) && ((!binaryValues[0]) || (!binaryValues[1]) || (!binaryValues[2]) || (!binaryValues[3]) || (!binaryValues[4]))) {
    fsm_LF.new_state = sm1_turn2; // Resume line-following when the line is detected
    robot.rel_theta = 0;
  } else if(fsm_LF.state == sm1_turn2 && (robot.rel_theta - turnAngle) >= 0){
    fsm_LF.new_state = sm1_lineFollowing;
  }

  set_state(fsm_LF, fsm_LF.new_state);

  // State actions
  if (fsm_LF.tis - fsm_LF.tup > interval) {
    fsm_LF.tup = fsm_LF.tis;

    if (fsm_LF.state == sm1_lineFollowing) {
      float w = robot_controller.followLinePID(binaryValues[0],binaryValues[1],binaryValues[2],binaryValues[3],binaryValues[4]);
      // Slow down when object is detected
      setRobotVW((sensorDist <= 0.2) ? 1.5 : robot_controller.vValues[robot_controller.mode], w);
    } else if (fsm_LF.state == sm1_turn1) {
      if(!binaryValues[2]){
        robot.rel_theta = 0;
      }
      setRobotVW(0,-30); // Turn in place
    } else if (fsm_LF.state == sm1_adjust1) {
      setRobotVW(0,-30); // Turn in place
    } else if (fsm_LF.state == sm1_move1) {
      float b = (a * sin(abs(turnAngle))) / (sin((PI/2) - abs(turnAngle)));
      float w = robot_controller.followEllipse(a, b, turnAngle, robot_controller.vValues[robot_controller.mode]);
      setRobotVW(robot_controller.vValues[robot_controller.mode], w);
    }else if(fsm_LF.state == sm1_turn2){
      setRobotVW(0, -30); // Turn in place
    }else if(fsm_LF.state == sm1_stop){
      setRobotVW(0, 0);
    }

    read_encoders();
    robot.enc1 = enc1;
    robot.enc2 = -enc2;
    robot.odometry();
  }
}

// Coords with objects
int objects[GRID_ROWS][GRID_COLS] = {};

void controlRobotGMSStm() {

  fsm_GMS.tis = millis();

  int finalX = 5;
  int finalY = 4;

  // State transitions
  if(fsm_GMS.state == sm2_scan && (binaryValues[0]+binaryValues[1]+binaryValues[2]+binaryValues[3]+binaryValues[4]) >= 3 && robot.rel_s >= 0.05){
    if(x == finalX && y == finalY){
      fsm_GMS.new_state = sm2_stop;
    }else{
      int nextDirection = robot_controller.calculateNextMove(x,y,currentDirection,objects,finalX,finalY); // Calculate next Direction
      fsm_GMS.new_state = calculateTurnState(currentDirection, nextDirection); // Calculate turn state
      currentDirection = nextDirection; // Update current direction
    }
  }else if(fsm_GMS.state == sm2_turnRight && (robot.rel_theta <= -(2.5 / 2)) && !binaryValues[2]){
    fsm_GMS.new_state = sm2_lineFollowing;
  }else if(fsm_GMS.state == sm2_turnLeft && (robot.rel_theta >= (2.5 / 2)) && !binaryValues[2]){
    fsm_GMS.new_state = sm2_lineFollowing;
  }else if(fsm_GMS.state == sm2_turnBack && (robot.rel_theta >= 2.5) && !binaryValues[2]){
    fsm_GMS.new_state = sm2_lineFollowing;
  }else if(fsm_GMS.state == sm2_goBack && ((binaryValues[0]+binaryValues[1]+binaryValues[2]+binaryValues[3]+binaryValues[4]) < 3)){
    fsm_GMS.new_state = sm2_scan;
    robot.rel_s = 0;
    robot.rel_theta = 0;
  }else if(fsm_GMS.state == sm2_lineFollowing && sensorDist <= 0.1){ // Object detected
    fsm_GMS.new_state = sm2_goBack;
    // Add to objects
    int tx = x + (currentDirection == RIGHT) - (currentDirection == LEFT);
    int ty = y + (currentDirection == UP) - (currentDirection == DOWN);
    objects[ty][tx] = 1;
    robot.rel_theta = 0;
  }else if(fsm_GMS.state == sm2_lineFollowing && ((binaryValues[0]+binaryValues[1]+binaryValues[2]+binaryValues[3]+binaryValues[4]) < 3)){
    fsm_GMS.new_state = sm2_scan;
    robot.rel_s = 0;
    robot.rel_theta = 0;
    x += (currentDirection == RIGHT) - (currentDirection == LEFT);
    y += (currentDirection == UP) - (currentDirection == DOWN);
  }

  set_state(fsm_GMS, fsm_GMS.new_state);

  // State actions
  if (fsm_GMS.tis - fsm_GMS.tup > interval) {
    fsm_GMS.tup = fsm_GMS.tis;
    if(fsm_GMS.state == sm2_lineFollowing || fsm_GMS.state == sm2_scan){
      float w = robot_controller.followLinePID(binaryValues[0],binaryValues[1],binaryValues[2],binaryValues[3],binaryValues[4]);
      setRobotVW((sensorDist <= 0.17) ? 1.5 : robot_controller.vValues[robot_controller.mode], w);
    }else if(fsm_GMS.state == sm2_goBack){
      float w = robot_controller.followLinePID(binaryValues[0],binaryValues[1],binaryValues[2],binaryValues[3],binaryValues[4]);
      setRobotVW(-robot_controller.vValues[robot_controller.mode], w);
    }else if(fsm_GMS.state == sm2_turnRight){
      setRobotVW(0, -30);
    }else if(fsm_GMS.state == sm2_turnLeft || fsm_GMS.state == sm2_turnBack){
      setRobotVW(0, 30);
    }else if(fsm_GMS.state == sm2_stop){
      setRobotVW(0, 0);
    }

    read_encoders();
    robot.enc1 = enc1;
    robot.enc2 = -enc2;
    robot.odometry();
  }
}

int calculateTurnState(int currentDirection, int nextDirection) {
    // Calculate turn direction considering the circular nature of directions
    int turnDirection = (nextDirection - currentDirection + 4) % 4;

    // Map turn direction to states
    if (turnDirection == 0) {
        return sm2_lineFollowing; // No turn needed
    } else if (turnDirection == 1) {
        return sm2_turnRight; // 90 degrees clockwise
    } else if (turnDirection == 3) {
        return sm2_turnLeft; // 90 degrees counterclockwise
    } else if (turnDirection == 2) {
        return sm2_turnBack; // 180 degrees turn
    }

    // Invalid turn direction (should not happen in normal cases)
    return -1;
}

void displayInfo() {

    String line1 = "enc1: " + String(robot.enc1) + ", enc2: " + String(robot.enc2);
    String line2 = "Channels: " + String(binaryValues[0]) + String(binaryValues[1]) + String(binaryValues[2]) + String(binaryValues[3]) + String(binaryValues[4]);
    String line9 = "Channels Values: " + String(sensorValues[0])+ " " + String(sensorValues[1]) + " " + String(sensorValues[2]) + " " + String(sensorValues[3]) + " " + String(sensorValues[4]);
    String line3 = "Dist: " + String(sensorDist, 5) + " m";
    String line4 = "Motor 1: " + String(robot.PWM_1) + ", Motor 2: " + String(robot.PWM_2);
    String line5 = "Battery: " + String(robot.battery_voltage) + " V";
    String line6 = "State: " + String(fsm_GMS.state);
    String line7 = "Kp,Ki,Kd: " + String(robot_controller.kpValues[robot_controller.mode]) 
    + "," + String(robot_controller.kiValues[robot_controller.mode]) 
    + "," + String(robot_controller.kdValues[robot_controller.mode]) ;
    String line8 = "Error: " + String(robot_controller.previous_error);

    if (currentMicros - lastPrintTime >= 200) {
      lastPrintTime = currentMicros;
      Serial.println("Ip address: " + WiFi.localIP().toString());
      Serial.println(line1);
      Serial.println(line2);
      Serial.println(line9);
      Serial.println(line3);
      Serial.println(line4);
      Serial.println(line5);
      Serial.println(line6);
      Serial.println(line7);
      Serial.println(line8);
      currentClient.println(line1);
      currentClient.println(line2);
      currentClient.println(line3);
      currentClient.println(line4);
      currentClient.println(line5);
      currentClient.println(line6);
      currentClient.println(line8);
    }
}

void receiveData() {
  if (currentClient.available()) {
      String data = currentClient.readStringUntil('\n'); // Read data until newline character
      Serial.println("Received data: " + data);          // Debug: Print the received data

      // Parse the incoming data as Kp, Ki, Kd
      int kpIndex = data.indexOf(',');
      int kiIndex = data.indexOf(',', kpIndex + 1);
      int kdIndex = data.indexOf(',', kiIndex + 1);
      int VelIndex = data.indexOf(',', kdIndex + 1);

      if (kpIndex > 0 && kiIndex > kpIndex) {
          // Extract Kp, Ki, Kd values from the string
          String kpStr = data.substring(0, kpIndex);
          String kiStr = data.substring(kpIndex + 1, kiIndex);
          String kdStr = data.substring(kiIndex + 1, kdIndex);
          String velStr = data.substring(kdIndex + 1, VelIndex);

          // Convert strings to floats and update PID values
          robot_controller.kpValues[robot_controller.mode] = kpStr.toFloat();
          robot_controller.kiValues[robot_controller.mode] = kiStr.toFloat();
          robot_controller.kdValues[robot_controller.mode] = kdStr.toFloat();
          robot_controller.vValues[robot_controller.mode] = velStr.toFloat();

          Serial.println("Updated PID values:");
          Serial.println("Kp: " + String(robot_controller.kpValues[robot_controller.mode]));
          Serial.println("Ki: " + String(robot_controller.kiValues[robot_controller.mode]));
          Serial.println("Kd: " + String(robot_controller.kdValues[robot_controller.mode]));
          Serial.println("Velocity: " + String(robot_controller.vValues[robot_controller.mode]));
      } else {
          Serial.println("Invalid data received: " + data);
      }
  }
}

void setRobotVW(float Vnom, float Wnom)
{
  // Calc outputs
  robot.v = Vnom;
  robot.w = Wnom;
  robot.VWToMotorsVoltage();

  setMotorPWM(robot.PWM_1, D1, D0);
  setMotorPWM(robot.PWM_2, D3, D2);
}

void setMotorPWM(int new_PWM, int pin_a, int pin_b)
{
  int PWM_max = 200;
  if (new_PWM >  PWM_max) new_PWM =  PWM_max;
  if (new_PWM < -PWM_max) new_PWM = -PWM_max;
  
  if (new_PWM == 0) {  // Both outputs 0 -> A = H, B = H
    analogWrite(pin_a, 255);
    analogWrite(pin_b, 255);
  } else if (new_PWM > 0) {
    analogWrite(pin_a, 255 - new_PWM);
    analogWrite(pin_b, 255);
  } else {
    analogWrite(pin_a, 255);
    analogWrite(pin_b, 255 + new_PWM);
  }
}

bool timer_handler(struct repeating_timer *t)
{
  int next_state, table_input, pins;
  digitalWriteFast(TEST_PIN, 1);

  pins = sio_hw->gpio_in;  // Just one read to get all pins

  next_state = pinIsHigh(ENCA1, pins) << 1;
  next_state |= pinIsHigh(ENCB1, pins);

  table_input = (encoder1_state << 2) | next_state;
  encoder1_pos += encoder_table[table_input];
  encoder1_state = next_state;

  next_state = pinIsHigh(ENCA2, pins) << 1;
  next_state |= pinIsHigh(ENCB2, pins);
  
  table_input = (encoder2_state << 2) | next_state;
  encoder2_pos -= encoder_table[table_input];
  encoder2_state = next_state;

  count++;
  digitalWriteFast(TEST_PIN, 0);
  return true;
}

void read_encoders(void)
{
  noInterrupts();  // Must be done with the interrupts disabled
  
  enc1 = encoder1_pos;
  enc2 = encoder2_pos;
  act_count = count;

  encoder1_pos = 0;
  encoder2_pos = 0;
  count = 0;

  interrupts();
}
