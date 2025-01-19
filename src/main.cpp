#include <Arduino.h>

#include "pico/cyw43_arch.h"
#include <WiFi.h>
#include <Wire.h>
#include <VL53L0X.h>
#include "robot.h"
#include "robot_controller.h"
#include <RPi_Pico_TimerInterrupt.h>

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
#define CHANNEL_1 0 // GPIO0
#define CHANNEL_2 1 // GPIO1
#define CHANNEL_3 2 // GPIO2
#define CHANNEL_4 3 // GPIO3
#define CHANNEL_5 4 // GPIO4

#define BUTTON_START 10  // Pin for start button

#define digitalWriteFast(pin, val)  (val ? sio_hw->gpio_set = (1 << pin) : sio_hw->gpio_clr = (1 << pin))
#define digitalReadFast(pin)        (((1 << pin) & sio_hw->gpio_in) >> pin)

#define pinIsHigh(pin, pins)        (((1 << pin) & pins) >> pin)

// fsm1 states
enum {
  sm1_lineFollowing = 0,
  sm1_turn1,
  sm1_adjust1,
  sm1_move1,
  sm1_turn2,
  sm1_move2,
};

VL53L0X tof;

robot_t robot;

robot_controller_t robot_controller;

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

fsm_t fsm;

int LED_state;
int ledCount = 0;

// WiFi credentials
const char* ssid = "NOS-676B";       // Replace with your WiFi SSID
const char* password = "L4N9U7JC"; // Replace with your WiFi password

WiFiClient currentClient = server.available();

// Time variables
unsigned long currentMicros, previousMicros;

//Distance sensor variables
float sensorDist, prev_sensorDist;

// Line following Sensor variables
int ch1, ch2, ch3, ch4, ch5;

int enc1, enc2;

volatile int encoder1_pos = 0;
volatile int encoder2_pos = 0;

int encoder1_state, encoder2_state;
// This table implements the counting event for every transition of the encoder state machine
int encoder_table[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0}; 

volatile int count;
int act_count;

unsigned long interval, last_cycle;

float Vinit = 3;

void read_encoders();
bool timer_handler(struct repeating_timer *t);
void displayInfo();
void controlRobotStm();
void set_state(fsm_t& fsm, int new_state);
void setRobotVW(float Vnom, float Wnom);
void receiveData();

void setup() {
  Serial.begin(9600);

  analogReadResolution(12); // Set ADC resolution to 12 bits

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

  pinMode(D0, OUTPUT);
  pinMode(D1, OUTPUT);
  pinMode(D2, OUTPUT);
  pinMode(D3, OUTPUT);

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
  //tof.setMeasurementTimingBudget(20000);

  // Start new distance measure
  tof.startReadRangeMillimeters();  

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

  interval = 40;             // In miliseconds
  robot.dt = 1e-3 * interval; // In seconds
  robot.PID1.dt = robot.dt;
  robot.PID2.dt = robot.dt;
  robot.control_mode = cm_pid;

  set_state(fsm, sm1_lineFollowing); 

  Serial.println("Starting path following...");

}

void loop() {

  if(!currentClient){
    currentClient = server.available();
    if (currentClient) {
      Serial.println("Client connected.");
    }
  }

  ch1 = digitalRead(CHANNEL_1);
  ch2 = digitalRead(CHANNEL_2);
  ch3 = digitalRead(CHANNEL_3);
  ch4 = digitalRead(CHANNEL_4);
  ch5 = digitalRead(CHANNEL_5);
  
  currentMicros = millis();

  // Call receiveData
  if (currentClient) {
    receiveData();
  }

  displayInfo();

  if(currentMicros % 40){
    if (tof.readRangeAvailable()) {
      prev_sensorDist = sensorDist;
      sensorDist = tof.readRangeMillimeters() * 1e-3;
    }
    tof.startReadRangeMillimeters();
  }

  robot.updateVoltage();

  if(robot.battery_voltage < 6.0){
    robot.PWM_1 = 0;
    robot.PWM_2 = 0;
    robot_controller.setMotorPWM(robot.PWM_1, D1, D0);
    robot_controller.setMotorPWM(robot.PWM_2, D3, D2);
    if(currentMicros % 500 == 0){
      LED_state = !LED_state;
      cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, LED_state);
    }
  }else{
    controlRobotStm();
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
int objAvoidVel = 100;

void controlRobotStm() {
  fsm.tis = millis();

  // State transitions
  if ((fsm.state == sm1_lineFollowing || fsm.state == sm1_move1 || fsm.state == sm1_turn2) && sensorDist < 0.1) {
    fsm.new_state = sm1_turn1; // Start turning
    robot.rel_theta = 0;
  } else if (fsm.state == sm1_turn1 && sensorDist > 0.15) {
    fsm.new_state = sm1_adjust1; // Move to adjust state
  } else if (fsm.state == sm1_adjust1 && (fsm.tis - fsm.tes) > 200) {
    fsm.new_state = sm1_move1; // Move forward slightly
    turnAngle = robot.rel_theta;
  } else if (fsm.state == sm1_move1 && ((!ch1) || (!ch2) || (!ch3) || (!ch4) || (!ch5))) {
    fsm.new_state = sm1_turn2; // Resume line-following when the line is detected
    robot.rel_theta = 0;
  } else if(fsm.state == sm1_turn2 && (robot.rel_theta - turnAngle) > 0){
    fsm.new_state = sm1_lineFollowing;
  }

  set_state(fsm, fsm.new_state);

  // State actions
  if (fsm.tis - fsm.tup > interval) {
    fsm.tup = fsm.tis;

    if (fsm.state == sm1_lineFollowing) {
      if(sensorDist < 0.2){
        robot_controller.v = Vinit * 0.3;
      }else{
        robot_controller.v = Vinit;
      }
      float w = robot_controller.followLinePID(ch1, ch2, ch3, ch4, ch5);
      setRobotVW(robot_controller.v, w);
    } else if (fsm.state == sm1_turn1) {
      robot_controller.setMotorPWM(objAvoidVel, D1, D0); // Turn in place
      robot_controller.setMotorPWM(-objAvoidVel, D3, D2);
    } else if (fsm.state == sm1_adjust1) {
      robot_controller.setMotorPWM(objAvoidVel, D1, D0); // Turn in place
      robot_controller.setMotorPWM(-objAvoidVel, D3, D2);
    } else if (fsm.state == sm1_move1) {
      float b = 0.10 / cos(turnAngle);
      robot_controller.followEllipse(0.10, b, turnAngle); // Move forward slightly
    }else if(fsm.state == sm1_turn2){
      robot_controller.setMotorPWM(objAvoidVel, D1, D0); // Turn in place
      robot_controller.setMotorPWM(-objAvoidVel, D3, D2);
    }
  }

  read_encoders();
  robot.enc1 = enc1;
  robot.enc2 = enc2;
  robot.odometry();
}

void displayInfo() {

    String line1 = "encoder1: " + String(enc1) + ", encoder2: " + String(enc2);
    String line2 = "Channels: " + String(ch1) + String(ch2) + String(ch3) + String(ch4) + String(ch5);
    String line3 = "Dist: " + String(sensorDist, 5) + " m";
    String line4 = "Motor 1: " + String(robot.PWM_1) + ", Motor 2: " + String(robot.PWM_2);
    String line5 = "Battery: " + String(robot.battery_voltage) + " V";
    String line6 = "State: " + String(fsm.state);
    String line7 = "Kp,Ki,Kd: " + String(robot_controller.kp) + "," + String(robot_controller.ki) + "," + String(robot_controller.kd) ;
    String line8 = "Error: " + String(robot_controller.previous_error);

    if (currentMicros % 2000 == 0) {
      Serial.println("Ip address: " + WiFi.localIP().toString());
      Serial.println(line1);
      Serial.println(line2);
      Serial.println(line3);
      Serial.println(line4);
      Serial.println(line5);
      Serial.println(line6);
      Serial.println(line7);
      Serial.println(line8);
    }  

    if(currentMicros % 200 == 0){
      currentClient.println(line1);
      currentClient.println(line2);
      currentClient.println(line3);
      currentClient.println(line4);
      currentClient.println(line5);
      currentClient.println(line6);
      currentClient.println(line7);
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
          robot_controller.kp = kpStr.toFloat();
          robot_controller.ki = kiStr.toFloat();
          robot_controller.kd = kdStr.toFloat();
          Vinit = velStr.toFloat();

          Serial.println("Updated PID values:");
          Serial.println("Kp: " + String(robot_controller.kp));
          Serial.println("Ki: " + String(robot_controller.ki));
          Serial.println("Kd: " + String(robot_controller.kd));
          Serial.println("Velocity: " + String(robot_controller.v));
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

  robot_controller.setMotorPWM(robot.PWM_1, D1, D0);
  robot_controller.setMotorPWM(robot.PWM_2, D3, D2);
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
