#include <AccelStepper.h>

// Define stepper motor connections
#define PAN_STEP_PIN  2
#define PAN_DIR_PIN   3
#define TILT_STEP_PIN 4
#define TILT_DIR_PIN  5

// Define endstop pins
#define PAN_LIMIT_MIN_PIN  9
#define PAN_LIMIT_MAX_PIN  10
#define TILT_LIMIT_MIN_PIN 11
#define TILT_LIMIT_MAX_PIN 12

// Create stepper instances
AccelStepper panStepper(AccelStepper::DRIVER, PAN_STEP_PIN, PAN_DIR_PIN);
AccelStepper tiltStepper(AccelStepper::DRIVER, TILT_STEP_PIN, TILT_DIR_PIN);

// System parameters
const long MAX_SPEED = 1000;          // Maximum speed in steps per second
const long HOMING_SPEED = 500;        // Speed during homing
const long MAX_ACCELERATION = 2000;    // Steps per second per second
const int SERIAL_UPDATE_MS = 50;      // Position feedback interval
const int COMMAND_TIMEOUT_MS = 250;   // Time before stopping if no commands received

// System state
unsigned long lastCommandTime = 0;     // Timestamp of last received command
unsigned long lastFeedbackTime = 0;    // Timestamp of last position feedback
bool isHomed = false;                 // Whether the system has been homed
long panRange = 0;                    // Total range of pan motion
long tiltRange = 0;                   // Total range of tilt motion
bool isHoming = false;                // Whether currently in homing sequence

// Communication buffer
const int BUFFER_SIZE = 2;
uint8_t cmdBuffer[BUFFER_SIZE];
int bufferIndex = 0;

// Command types
enum CommandType {
  CMD_VELOCITY = 0,
  CMD_HOME = 1,
  CMD_NEUTRAL = 2
};

void setup() {
  Serial.begin(115200);
  
  // Configure steppers
  configureStepper(panStepper);
  configureStepper(tiltStepper);
  
  // Configure endstop pins
  pinMode(PAN_LIMIT_MIN_PIN, INPUT_PULLUP);
  pinMode(PAN_LIMIT_MAX_PIN, INPUT_PULLUP);
  pinMode(TILT_LIMIT_MIN_PIN, INPUT_PULLUP);
  pinMode(TILT_LIMIT_MAX_PIN, INPUT_PULLUP);
  
  while (!Serial) {
    delay(10);
  }
}

void configureStepper(AccelStepper &stepper) {
  stepper.setMaxSpeed(MAX_SPEED);
  stepper.setAcceleration(MAX_ACCELERATION);
  stepper.setCurrentPosition(0);
}

bool performHoming() {
  static int homingState = 0;
  static unsigned long homingStartTime = 0;
  
  if (homingState == 0) {
    // Start homing sequence
    homingStartTime = millis();
    panStepper.setSpeed(-HOMING_SPEED);
    tiltStepper.setSpeed(-HOMING_SPEED);
    homingState = 1;
    return false;
  }
  
  // Check for timeout
  if (millis() - homingStartTime > 30000) {  // 30 second timeout
    homingState = 0;
    isHoming = false;
    return false;
  }
  
  switch(homingState) {
    case 1:  // Moving to min positions
      if (digitalRead(PAN_LIMIT_MIN_PIN) == LOW) {
        panStepper.setSpeed(0);
        panStepper.setCurrentPosition(0);
      }
      if (digitalRead(TILT_LIMIT_MIN_PIN) == LOW) {
        tiltStepper.setSpeed(0);
        tiltStepper.setCurrentPosition(0);
      }
      if (panStepper.speed() == 0 && tiltStepper.speed() == 0) {
        homingState = 2;
        panStepper.setSpeed(HOMING_SPEED);
        tiltStepper.setSpeed(HOMING_SPEED);
      }
      break;
      
    case 2:  // Moving to max positions to measure range
      if (digitalRead(PAN_LIMIT_MAX_PIN) == LOW) {
        panStepper.setSpeed(0);
        panRange = panStepper.currentPosition();
      }
      if (digitalRead(TILT_LIMIT_MAX_PIN) == LOW) {
        tiltStepper.setSpeed(0);
        tiltRange = tiltStepper.currentPosition();
      }
      if (panStepper.speed() == 0 && tiltStepper.speed() == 0) {
        // Move to neutral positions
        panStepper.moveTo(panRange / 2);
        tiltStepper.moveTo(tiltRange / 2);
        homingState = 3;
      }
      break;
      
    case 3:  // Moving to neutral positions
      if (!panStepper.isRunning() && !tiltStepper.isRunning()) {
        homingState = 0;
        isHomed = true;
        isHoming = false;
        return true;
      }
      break;
  }
  
  panStepper.runSpeed();
  tiltStepper.runSpeed();
  return false;
}

void moveToNeutral() {
  if (!isHomed) return;
  panStepper.moveTo(panRange / 2);
  tiltStepper.moveTo(tiltRange / 2);
}

void processCommand(uint8_t cmd, uint8_t data) {
  switch(cmd) {
    case CMD_VELOCITY:
      if (!isHoming) {
        float panVelocity = ((int)data - 128) * (MAX_SPEED / 127.0);
        panStepper.setSpeed(panVelocity);
      }
      break;
      
    case CMD_HOME:
      isHoming = true;
      break;
      
    case CMD_NEUTRAL:
      if (!isHoming) {
        moveToNeutral();
      }
      break;
  }
  lastCommandTime = millis();
}

void processSerial() {
  while (Serial.available() > 0 && bufferIndex < BUFFER_SIZE) {
    cmdBuffer[bufferIndex++] = Serial.read();
    
    if (bufferIndex == BUFFER_SIZE) {
      processCommand(cmdBuffer[0], cmdBuffer[1]);
      bufferIndex = 0;
    }
  }
}

void sendPositionFeedback() {
  if (millis() - lastFeedbackTime >= SERIAL_UPDATE_MS) {
    Serial.print("P:");
    Serial.print(panStepper.currentPosition());
    Serial.print(",T:");
    Serial.print(tiltStepper.currentPosition());
    Serial.print(",H:");
    Serial.println(isHomed ? 1 : 0);
    lastFeedbackTime = millis();
  }
}

void loop() {
  if (isHoming) {
    performHoming();
  } else {
    processSerial();
    
    // Run steppers in velocity mode
    if (panStepper.speed() != 0) {
      panStepper.runSpeed();
    }
    if (tiltStepper.speed() != 0) {
      tiltStepper.runSpeed();
    }
    
    // Run steppers in position mode
    panStepper.run();
    tiltStepper.run();
  }
  
  sendPositionFeedback();
}