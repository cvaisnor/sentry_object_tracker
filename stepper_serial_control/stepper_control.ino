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
      if (totalSteps < 0 || speed == 0 || steps == 0) {
        return;
      }
      
      if (currentStep + steps > totalSteps && direction == MotorDirection::Direction1) {
        // get difference between currentStep and totalSteps
        steps = totalSteps - currentStep;
      } 
      else if (steps > currentStep && direction == MotorDirection::Direction0) {
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

      const int BUFFER = 200;

      while(!endHit) {
        unsafeMove(1, MotorDirection::Direction0, 375);
        endHit = checkEndStops();
        if(endHit) {
          // Serial.println("Left endstop hit during left movement");
          
          // move back # steps to get off the end stop
          unsafeMove(BUFFER, MotorDirection::Direction1, 375);
        }
      }

      // Reset the endstop hit and start counting steps
      endHit = false;
      int steps = 0;

      // Serial.println("Switching directions");
      // Move to the right end, counting steps
      while(!endHit) {
        unsafeMove(1, MotorDirection::Direction1, 375);
        steps++;
        endHit = checkEndStops();
        if(endHit) {
          // Serial.println("Right endstop hit during right movement");
          
          // move back # steps to get off the end stop
          unsafeMove(BUFFER, MotorDirection::Direction0, 375);
          steps -= BUFFER;
        }
      }
      // write the total number of steps to the class attribute
      totalSteps = steps;
      currentStep = totalSteps;
    
      // set to neutral position
      moveToNeutral(375);    
    }

};

class MotorSpeed {
public:
  enum Speed : uint8_t {
    Off = 0,
    Speed1,
    Speed2,
    Speed3,
    Speed4,
    Speed5,
    Speed6,
    Speed7
  };

  MotorSpeed()
    : m_speed(Off){};
  MotorSpeed(Speed speed)
    : m_speed(speed){};
  MotorSpeed(uint8_t speed) {
    if (speed >= Off && speed <= Speed7) {
      m_speed = (Speed)speed;
    } else {
      m_speed = Off;
    }
  };

  bool operator==(MotorSpeed rhs) {
      return m_speed == rhs.m_speed;
  };
  bool operator!=(MotorSpeed rhs) {
      return m_speed != rhs.m_speed;
  };
  bool operator<(MotorSpeed rhs) {
      return m_speed < rhs.m_speed;
  };
  bool operator>(MotorSpeed rhs) {
      return m_speed > rhs.m_speed;
  };

  int getDelay() {
    switch (m_speed) {
      case Speed1: return 2000;
      case Speed2: return 1500;
      case Speed3: return 1000;
      case Speed4: return 500;
      case Speed5: return 375;
      case Speed6: return 250;
      case Speed7: return 175;
      default: return 0;
    }
  };

private:
  Speed m_speed;
};

struct MotorsState {
  MotorDirection panDirection;
  MotorDirection tiltDirection;
  MotorSpeed panSpeed;
  MotorSpeed tiltSpeed;

  MotorsState(uint8_t command) {
    this->panDirection = (MotorDirection)bitRead(command, 7);
    this->tiltDirection = (MotorDirection)bitRead(command, 6);
    this->panSpeed = MotorSpeed((command & 0b111000) >> 3);
    this->tiltSpeed = MotorSpeed(command & 0b111);
  }

  MotorsState() {
    this->panDirection = (MotorDirection)0;
    this->tiltDirection = (MotorDirection)0;
    this->panSpeed = MotorSpeed();
    this->tiltSpeed = MotorSpeed();
  }
};

enum class MessageCommand : uint8_t {
  Calibrate = 0, 
  Neutral = 1,
  Move = 2,
};


struct Message {
  MessageCommand command;
  MotorsState motorsState;

  Message(MessageCommand command, MotorsState motorsState) {
    this->command = command;
    this->motorsState = motorsState;
  }
  
  Message(MessageCommand command) {
    this->command = command;
  }
};


Message readMessage() {
  uint8_t command = Serial.read();
  if (command == 0) {
    return Message(MessageCommand::Calibrate);
  }
  else if (command == 1) {
    return Message(MessageCommand::Neutral);
  }
  else if (command == 2) {
    while (!Serial.available()); // wait until next byte arrives
    command = Serial.read();
    return Message(MessageCommand::Move, MotorsState(command));
  }
}

// struct for acknowledgement message
enum class Acknowledgement : uint8_t {
  Calibrate = 0,
  Neutral = 1,
  Move = 2
};

// struct for message status
enum class MessageStatus : uint8_t {
  Success = 0,
  Error = 1
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
  Serial.begin(115200);

  pinMode(PAN_STOP_PIN, INPUT);
  pinMode(TILT_STOP_PIN, INPUT);

  // lastCommandRead = millis();
  digitalWrite(PAN_STOP_PIN, HIGH);
  digitalWrite(TILT_STOP_PIN, HIGH);
}

void loop() {
  if (Serial.available()) {
    // read the message from the serial port
    Message message = readMessage();
    // calibrate the motors
    if (message.command == MessageCommand::Calibrate) {
      Serial.write((uint8_t)Acknowledgement::Calibrate);
      panMotor.calibrate();
      tiltMotor.calibrate();
      Serial.write((uint8_t)MessageStatus::Success);
    } // move to neutral position
    else if (message.command == MessageCommand::Neutral) {
      Serial.write((uint8_t)Acknowledgement::Neutral);
      panMotor.moveToNeutral(500);
      tiltMotor.moveToNeutral(500);
      Serial.write((uint8_t)MessageStatus::Success);
    } // move the motors
    else if (message.command == MessageCommand::Move) {
      Serial.write((uint8_t)Acknowledgement::Move);
      if (message.motorsState.panSpeed.getDelay() > 0) {
        panMotor.move((16000/message.motorsState.panSpeed.getDelay()), message.motorsState.panDirection, message.motorsState.panSpeed.getDelay());
      }
      if (message.motorsState.tiltSpeed.getDelay() > 0) {     
        tiltMotor.move((16000/message.motorsState.tiltSpeed.getDelay()), message.motorsState.tiltDirection, message.motorsState.tiltSpeed.getDelay());
      }
      Serial.write((uint8_t)MessageStatus::Success);
    }
  }
}