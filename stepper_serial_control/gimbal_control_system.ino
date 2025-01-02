#include <AccelStepper.h>
#include <Servo.h>

// Define stepper motor connections
#define PAN_STEP_PIN  2
#define PAN_DIR_PIN   3
#define TILT_STEP_PIN 5
#define TILT_DIR_PIN  6

Servo zoomServo;  // create servo object

// Define endstop pins
#define PAN_LIMIT_PIN  9
#define TILT_LIMIT_PIN 10

// Create stepper instances
AccelStepper panStepper(AccelStepper::DRIVER, PAN_STEP_PIN, PAN_DIR_PIN);
AccelStepper tiltStepper(AccelStepper::DRIVER, TILT_STEP_PIN, TILT_DIR_PIN);

// System parameters
const long MAX_SPEED = 4000;          // Maximum speed in steps per second
const long HOMING_SPEED = 2000;        // Speed during homing
const long MAX_ACCELERATION = 3000;    // Steps per second per second
const int SERIAL_UPDATE_MS = 50;      // Position feedback interval
const int COMMAND_TIMEOUT_MS = 250;   // Time before stopping if no commands received
const int VELOCITY_TO_DISTANCE = 1000; // How far to move based on velocity (adjust as needed)

// System state
unsigned long lastCommandTime = 0;     // Timestamp of last received command
unsigned long lastFeedbackTime = 0;    // Timestamp of last position feedback
bool isAtPanMin = false;              // At pan minimum endstop
bool isAtPanMax = false;              // At pan maximum endstop
bool isAtTiltMin = false;             // At tilt minimum endstop
bool isAtTiltMax = false;             // At tilt maximum endstop

// Communication buffer
const int BUFFER_SIZE = 4;
uint8_t cmdBuffer[BUFFER_SIZE];
int bufferIndex = 0;

// Command types
enum CommandType {
  CMD_VELOCITY = 0,
  CMD_NEUTRAL = 1
};

void setup() {
  Serial.begin(115200);
  
  // Configure steppers
  configureStepper(panStepper);
  configureStepper(tiltStepper);
  
  // Configure endstop pins
  pinMode(PAN_LIMIT_PIN, INPUT_PULLUP);
  pinMode(TILT_LIMIT_PIN, INPUT_PULLUP);
  
  // Attach servo to pin 9
  zoomServo.attach(7);
  zoomServo.write(0);

  while (!Serial) {
    delay(10);
  }
}

void configureStepper(AccelStepper &stepper) {
  stepper.setMaxSpeed(MAX_SPEED);
  stepper.setAcceleration(MAX_ACCELERATION);
  stepper.setCurrentPosition(0);
}

void moveToNeutral() {
  panStepper.moveTo(0);
  tiltStepper.moveTo(0);
}

// Function to convert velocity command to position target
void velocityToPosition(float velocity, AccelStepper &stepper, bool isMin, bool isMax) {
  if (abs(velocity) < 1.0) {
    stepper.stop();
    return;
  }
  
  // Check endstops - prevent movement in blocked directions
  if ((velocity < 0 && isMin) || (velocity > 0 && isMax)) {
    stepper.stop();
    return;
  }
  
  // Calculate move distance based on velocity
  long moveDistance = (abs(velocity) / MAX_SPEED) * VELOCITY_TO_DISTANCE;
  
  // Set direction based on velocity sign
  long currentPos = stepper.currentPosition();
  long newTarget = velocity > 0 ? currentPos + moveDistance : currentPos - moveDistance;
  
  // Set movement speed based on velocity magnitude
  float speed = abs(velocity);
  if (speed > MAX_SPEED) speed = MAX_SPEED;
  stepper.setMaxSpeed(speed);
  
  stepper.moveTo(newTarget);
}

void processVelocityCommand(uint8_t panByte, uint8_t tiltByte) {
  // Convert from byte (0-255) to signed velocity (-MAX_SPEED to +MAX_SPEED)
  float panVelocity = (((int)panByte - 128) / 127.0) * MAX_SPEED;
  float tiltVelocity = (((int)tiltByte - 128) / 127.0) * MAX_SPEED;
  
  velocityToPosition(panVelocity, panStepper, isAtPanMin, isAtPanMax);
  velocityToPosition(tiltVelocity, tiltStepper, isAtTiltMin, isAtTiltMax);
}

void processCommand(uint8_t cmd, uint8_t data1, uint8_t data2, uint8_t data3) {
  switch(cmd) {
    case CMD_VELOCITY:
      processVelocityCommand(data1, data2);
      processZoomServo(data3);
      break;
      
    case CMD_NEUTRAL:
      moveToNeutral();
      break;
  }
  lastCommandTime = millis();
}

void processZoomServo(uint8_t zoomServoByte) {
  // Convert from byte (0-255) to angle (0-180)
  int angle = map(zoomServoByte, 0, 255, 0, 180);
  zoomServo.write(angle);
}

void processSerial() {
  // Wait until we have at least 4 bytes available
  if (Serial.available() >= 4) {
    // Clear the input buffer first
    while (Serial.available() > 4) {
      Serial.read();
    }
    
    // Read exactly 4 bytes
    uint8_t cmd = Serial.read(); // Command byte
    uint8_t data1 = Serial.read(); // pan
    uint8_t data2 = Serial.read(); // tilt
    uint8_t data3 = Serial.read(); // zoomServo
    
    // Debug received command
    Serial.print("Received command bytes: [");
    Serial.print(cmd);
    Serial.print(", ");
    Serial.print(data1);
    Serial.print(", ");
    Serial.print(data2);
    Serial.print(", ");
    Serial.print(data3);
    Serial.println("]");
    
    processCommand(cmd, data1, data2, data3);
  }
}

void sendPositionFeedback() {
  if (millis() - lastFeedbackTime >= SERIAL_UPDATE_MS && Serial.availableForWrite() >= 30) {
    Serial.print("P:");
    Serial.print(panStepper.currentPosition());
    Serial.print(",T:");
    Serial.print(tiltStepper.currentPosition());
    Serial.print(",Z:");
    Serial.print(zoomServo.read());
    lastFeedbackTime = millis();
  }
}

void checkEndstops() {
  // Update endstop states
  isAtPanMin = digitalRead(PAN_LIMIT_PIN) == LOW && panStepper.speed() < 0;
  isAtPanMax = digitalRead(PAN_LIMIT_PIN) == LOW && panStepper.speed() > 0;
  isAtTiltMin = digitalRead(TILT_LIMIT_PIN) == LOW && tiltStepper.speed() < 0;
  isAtTiltMax = digitalRead(TILT_LIMIT_PIN) == LOW && tiltStepper.speed() > 0;
  
  // Stop motion if at endstops
  if (isAtPanMin || isAtPanMax) {
    panStepper.stop();
  }
  if (isAtTiltMin || isAtTiltMax) {
    tiltStepper.stop();
  }
}

void loop() {
  processSerial();
  checkEndstops();
  
  // Run steppers with acceleration
  panStepper.run();
  tiltStepper.run();
  
  sendPositionFeedback();
}