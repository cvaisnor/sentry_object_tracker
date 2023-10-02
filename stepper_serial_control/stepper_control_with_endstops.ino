const int PAN_STEP_PIN = 2;
const int PAN_DIR_PIN = 5;
const int TILT_STEP_PIN = 3;
const int TILT_DIR_PIN = 6;
const int PAN_STOP_PIN = 9;
const int TILT_STOP_PIN = 10;

enum class MotorDirection : uint8_t {
  Direction0 = 0, // towards 0 steps
  Direction1 = 1 // towards totalSteps
};

class StepperMotor {

  int totalSteps;
  int currentStep; // this is the current step the motor is on out of totalSteps
  int stepPin;
  int dirPin;
  int stopPin;

  void unsafeMove(unsigned int steps, MotorDirection direction, unsigned int speed) {
    digitalWrite(dirPin, (uint8_t)direction);

    for(int i = 0; i < steps; i++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(speed);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(speed);
    }
  }

  bool checkEndStops() {
    return digitalRead(stopPin) == LOW;
  }

  public:
    StepperMotor(int stepPin, int dirPin, int stopPin) {
      this->stepPin = stepPin;
      this->dirPin = dirPin;
      this->stopPin = stopPin;
      this->totalSteps = -1;
      this->currentStep = 0;
    }

    void move(unsigned int steps, MotorDirection direction, unsigned int speed) {
      if (totalSteps < 0) {
        return;
      }
      
      if (currentStep + steps > totalSteps && direction == MotorDirection::Direction1) {
        // get difference between currentStep and totalSteps
        steps = totalSteps - currentStep;
      } 
      else if (currentStep - steps < 0 && direction == MotorDirection::Direction0) {
        // get difference between currentStep and 0
        steps = currentStep;
      }
      
      unsafeMove(steps, direction, speed);
      
      if (direction == MotorDirection::Direction0) {
        currentStep -= steps;
      }
      else if (direction == MotorDirection::Direction1) {
        currentStep += steps;
      }
    }

    void moveTo(unsigned int step, unsigned int speed) {
      if(step > currentStep) {
        move(step - currentStep, MotorDirection::Direction1, speed);
      }
      else if(step < currentStep) {
        move(currentStep - step, MotorDirection::Direction0, speed);
      }
    }

    void moveToNeutral(unsigned int speed) {
      moveTo(totalSteps / 2, speed);
    }

    void calibrate() {
      // Move to the left end
      bool endHit = false;

      while(!endHit) {
        unsafeMove(1, MotorDirection::Direction0, 500);
        endHit = checkEndStops();
        if(endHit) {
          Serial.println("Left endstop hit during left movement");
          
          // move back # steps to get off the end stop
          unsafeMove(200, MotorDirection::Direction1, 500);
        }
      }

      // Reset the endstop hit and start counting steps
      endHit = false;
      int steps = 0;

      Serial.println("Switching directions");
      // Move to the right end, counting steps
      while(!endHit) {
        unsafeMove(1, MotorDirection::Direction1, 500);
        endHit = checkEndStops();
        if(endHit) {
          Serial.println("Right endstop hit during right movement");
          
          // move back # steps to get off the end stop
          unsafeMove(200, MotorDirection::Direction0, 500);
          steps -= 200;
          }
        else {
          steps++;
        }
      }
      // write the total number of steps to the class attribute
      totalSteps = steps;
      currentStep = totalSteps;
    
      // set to neutral position
      moveToNeutral(500);    
    }

};

// initialize the pan and tilt motors
StepperMotor panMotor = StepperMotor(PAN_STEP_PIN, PAN_DIR_PIN, PAN_STOP_PIN);
StepperMotor tiltMotor = StepperMotor(TILT_STEP_PIN, TILT_DIR_PIN, TILT_STOP_PIN);

void setup() {
  // Initialize your stepper motor driver logic here
  pinMode(PAN_STEP_PIN, OUTPUT);
  pinMode(PAN_DIR_PIN, OUTPUT);
  pinMode(TILT_STEP_PIN, OUTPUT);
  pinMode(TILT_DIR_PIN, OUTPUT);
  Serial.begin(9600);

  pinMode(PAN_STOP_PIN, INPUT);
  pinMode(TILT_STOP_PIN, INPUT);

  // lastCommandRead = millis();
  digitalWrite(PAN_STOP_PIN, HIGH);
  digitalWrite(TILT_STOP_PIN, HIGH);
}

void loop() {
  delay(3000);
  Serial.println("Calibrating");
  panMotor.calibrate();
  delay(1000);
  tiltMotor.calibrate();
  delay(20000);
}