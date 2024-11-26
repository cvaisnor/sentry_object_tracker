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
bool isHomed = false;                 // Whether the system has been homed
long panRange = 0;                    // Total range of pan motion
long tiltRange = 0;                   // Total range of tilt motion
bool isHoming = false;                // Whether currently in homing sequence

// Communication buffer
const int BUFFER_SIZE = 4;
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
  pinMode(PAN_LIMIT_PIN, INPUT_PULLUP);
  pinMode(TILT_LIMIT_PIN, INPUT_PULLUP);
  
  // Attach servo to pin 9
  zoomServo.attach(7);
  zoomServo.write(90);

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
  if (millis() - homingStartTime > 45000) {  // 45 second timeout
    homingState = 0;
    isHoming = false;
    return false;
  }
  
  switch(homingState) {
    case 1:  // Moving to min positions
      if (digitalRead(PAN_LIMIT_PIN) == LOW) {
        panStepper.setSpeed(0);
      }
      if (digitalRead(TILT_LIMIT_PIN) == LOW) {
        tiltStepper.setSpeed(0);
      }
      if (panStepper.speed() == 0 && tiltStepper.speed() == 0) {
        delay(100);  // Small delay to ensure we're stable at the limit
        panStepper.setCurrentPosition(0);
        tiltStepper.setCurrentPosition(0);
        panStepper.setSpeed(HOMING_SPEED/2); // switch directions to move off the endstops slowly
        tiltStepper.setSpeed(HOMING_SPEED/2);
        homingState++;
      }
      break;

    case 2:  // move off the endstops
      if (digitalRead(PAN_LIMIT_PIN) == HIGH) {
        panStepper.setSpeed(HOMING_SPEED); // Resume full speed once off limit
      }
      if (digitalRead(TILT_LIMIT_PIN) == HIGH) {
        tiltStepper.setSpeed(HOMING_SPEED);
      }
      if (panStepper.speed() == (HOMING_SPEED) && tiltStepper.speed() == (HOMING_SPEED)) {
        homingState++;
      }
      break;
    
    case 3: // move to the max positions
      if (digitalRead(PAN_LIMIT_PIN) == LOW) {
        panStepper.setSpeed(0);
      }
      if (digitalRead(TILT_LIMIT_PIN) == LOW) {
        tiltStepper.setSpeed(0);
      }
      if (panStepper.speed() == 0 && tiltStepper.speed() == 0) {
        delay(100); // Small delay to ensure we're stable at the limit
        panRange = panStepper.currentPosition();  // Record the range at max limit
        tiltRange = tiltStepper.currentPosition();  // Record the range at max limit

        // Send Range information to Python
        Serial.print("R:");
        Serial.print(panRange);
        Serial.print(",");
        Serial.println(tiltRange);

        // Calculate center positions
        long panCenter = panRange / 2;
        long tiltCenter = tiltRange / 2;
        // Move to center positions
        tiltStepper.setSpeed(-HOMING_SPEED);
        panStepper.setSpeed(-HOMING_SPEED);
        panStepper.moveTo(panCenter);
        tiltStepper.moveTo(tiltCenter);
        homingState++;
      }
      break;

    case 4: // Moving to neutral positions
      if (!panStepper.isRunning() && !tiltStepper.isRunning()) {
        homingState = 0;
        isHomed = true;
        isHoming = false;
        return true;
      }
      break;
  }

  // Run the steppers
  if (homingState == 4) {
    // Use position mode for final centering
    panStepper.run();
    tiltStepper.run(); // dynamic
  } else {
    // Use constant speed mode for homing
    panStepper.runSpeed(); // constant
    tiltStepper.runSpeed();
  }

  return false;
}

void moveToNeutral() {
  if (!isHomed) return;
  
  long panTarget = panRange / 2;
  long tiltTarget = tiltRange / 2;
  
  // Debug output
  // Serial.print("Moving to neutral - Pan target: ");
  // Serial.print(panTarget);
  // Serial.print(" Tilt target: ");
  // Serial.println(tiltTarget);
  
  panStepper.moveTo(panTarget);
  tiltStepper.moveTo(tiltTarget);
}

// Function to convert velocity command to position target
void velocityToPosition(float velocity, AccelStepper &stepper, long rangeLimit) {
  if (!isHomed) return;
  
  // Debug velocity input
  // Serial.print("Processing velocity: ");
  // Serial.println(velocity);
  
  if (abs(velocity) < 1.0) {
    stepper.stop();
    return;
  }
  
  // Get current position
  long currentPos = stepper.currentPosition();
  
  // Calculate move distance based on velocity
  // Increase VELOCITY_TO_DISTANCE if movement is too slow
  long moveDistance = (abs(velocity) / MAX_SPEED) * VELOCITY_TO_DISTANCE;
  
  // Set direction based on velocity sign
  long newTarget;
  if (velocity > 0) {
    newTarget = currentPos + moveDistance;
  } else {
    newTarget = currentPos - moveDistance;
  }
  
  // Constrain to valid range (0 to rangeLimit)
  if (rangeLimit > 0) {
    newTarget = constrain(newTarget, 200, rangeLimit-200);
  }
  
  // Debug target position
  // Serial.print("Current pos: ");
  // Serial.print(currentPos);
  // Serial.print(" New target: ");
  // Serial.println(newTarget);
  
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
  
  // Debug output
  // Serial.print("Received command bytes - Pan: ");
  // Serial.print(panByte);
  // Serial.print(" Tilt: ");
  // Serial.println(tiltByte);
  
  // Serial.print("Converted velocities - Pan: ");
  // Serial.print(panVelocity);
  // Serial.print(" Tilt: ");
  // Serial.println(tiltVelocity);
  
  velocityToPosition(panVelocity, panStepper, panRange);
  velocityToPosition(tiltVelocity, tiltStepper, tiltRange);
}

void processCommand(uint8_t cmd, uint8_t data1, uint8_t data2, uint8_t data3) {
  
  switch(cmd) {
    case CMD_VELOCITY:
      if (!isHoming) {
        processVelocityCommand(data1, data2);
        processZoomServo(data3);
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

void processZoomServo(uint8_t zoomServoByte) {
  // Convert from byte (0-255) to angle (0-180)
  int angle = map(zoomServoByte, 0, 255, 0, 180);
  
  // Debug output
  // Serial.print("Received zoom servo byte: ");
  // Serial.print(zoomServoByte);
  // Serial.print(" Angle: ");
  // Serial.println(angle);
  
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
    
    // Check endstops before running steppers
    if (digitalRead(PAN_LIMIT_PIN) == LOW) {
      // At limit - update position and stop
      if (panStepper.speed() > 0) {
        panStepper.setCurrentPosition(panRange);
      } else if (panStepper.speed() < 0) {
        panStepper.setCurrentPosition(0);
      }
      panStepper.stop();
    }
    
    if (digitalRead(TILT_LIMIT_PIN) == LOW) {
      // At limit - update position and stop
      if (tiltStepper.speed() > 0) {
        tiltStepper.setCurrentPosition(tiltRange);
      } else if (tiltStepper.speed() < 0) {
        tiltStepper.setCurrentPosition(0);
      }
      tiltStepper.stop();
    }
    
    // Run steppers with acceleration
    panStepper.run();
    tiltStepper.run();
  }
  
  sendPositionFeedback();
}