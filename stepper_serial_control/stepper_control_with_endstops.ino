const int PAN_STEP_PIN = 2;
const int PAN_DIR_PIN = 5;
const int TILT_STEP_PIN = 3;
const int TILT_DIR_PIN = 6;
const int PAN_END_STOP = 9;
const int TILT_END_STOP = 10;

enum class MotorDirection : uint8_t {
  Down = 0,
  Up = 1,
  Left = 0,
  Right = 1
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
  MotorSpeed(int speed) {
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
};

MotorsState MOTORS_STATE;
unsigned long lastCommandRead;

MotorDirection LAST_PAN_DIRECTION;
MotorDirection LAST_TILT_DIRECTION;
bool ENDSTOP_LEFT_HIT = false;
bool ENDSTOP_RIGHT_HIT = false;

int PAN_RANGE = 0;
int TILT_RANGE = 0;
int PAN_MIDDLE = 0;
int TILT_MIDDLE = 0;


void setEndStopPins(int PAN_END_STOP, int TILT_END_STOP) {
  digitalWrite(PAN_END_STOP, HIGH);
  digitalWrite(TILT_END_STOP, HIGH);
}


void setup() {
  // Initialize your stepper motor driver logic here
  pinMode(PAN_STEP_PIN, OUTPUT);
  pinMode(PAN_DIR_PIN, OUTPUT);
  pinMode(TILT_STEP_PIN, OUTPUT);
  pinMode(TILT_DIR_PIN, OUTPUT);
  Serial.begin(9600);

  pinMode(PAN_END_STOP, INPUT);
  pinMode(TILT_END_STOP, INPUT);

  lastCommandRead = millis();
  setEndStopPins(PAN_END_STOP, TILT_END_STOP);
}


void checkEndStops() {
  if(digitalRead(TILT_END_STOP) == LOW) {
    if(LAST_TILT_DIRECTION == MotorDirection::Left) {
      ENDSTOP_LEFT_HIT = true;
      Serial.println("Left endstop hit");
    }
    else {
      ENDSTOP_RIGHT_HIT = true;
      Serial.println("Right endstop hit");
    }
  }
}


void calibrateAxis(int stepPin, int dirPin, int &axisRange, int &axisMiddle) {
  // Move to the left end
  while(!ENDSTOP_LEFT_HIT) {
    digitalWrite(dirPin, (uint8_t)MotorDirection::Left);
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000);
    checkEndStops();

    if(ENDSTOP_LEFT_HIT) {
      Serial.println("Left endstop hit during left movement");
    }
  }

  // Reset the endstop hit and start counting steps
  ENDSTOP_LEFT_HIT = false;
  int steps = 0;

  // Move to the right end, counting steps
  while(!ENDSTOP_RIGHT_HIT) {
    digitalWrite(dirPin, (uint8_t)MotorDirection::Right);
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000);
    checkEndStops();
    if(ENDSTOP_RIGHT_HIT) {
      Serial.println("Right endstop hit during left movement");
    }
    steps++;
  }

  // Reset the endstop hit and set the axis range and middle
  ENDSTOP_RIGHT_HIT = false;
  axisRange = steps;
  axisMiddle = steps / 2;

  // Move to the middle position
  digitalWrite(dirPin, (uint8_t)MotorDirection::Left);
  for(int i = 0; i < axisMiddle; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000);
  }
}


void calibrateAxes() {
  // calibrateAxis(PAN_STEP_PIN, PAN_DIR_PIN, PAN_RANGE, PAN_MIDDLE); // calibrate pan axis
  calibrateAxis(TILT_STEP_PIN, TILT_DIR_PIN, TILT_RANGE, TILT_MIDDLE); // calibrate tilt axis
}


void stepMotors(MotorsState state, int distance) {
  checkEndStops();

  if (state.panSpeed == MotorSpeed::Off && state.tiltSpeed == MotorSpeed::Off) {
    return;
  }

  LAST_PAN_DIRECTION = state.panDirection;
  digitalWrite(PAN_DIR_PIN, (uint8_t)state.panDirection);
  digitalWrite(TILT_DIR_PIN, (uint8_t)state.tiltDirection);

  digitalWrite(PAN_DIR_PIN, (uint8_t)state.panDirection);
  digitalWrite(TILT_DIR_PIN, (uint8_t)state.tiltDirection);

  int maxDelay = max(state.panSpeed, state.tiltSpeed).getDelay();

  // TODO: control speed of each direction independently
  for (int i = 0; i < distance; i++) {
    if (state.panSpeed != MotorSpeed::Off) {
      digitalWrite(PAN_STEP_PIN, HIGH);
    }
    if (state.tiltSpeed != MotorSpeed::Off) {
      digitalWrite(TILT_STEP_PIN, HIGH);
    }
    delayMicroseconds(maxDelay);

    if (state.panSpeed != MotorSpeed::Off) {
      digitalWrite(PAN_STEP_PIN, LOW);
    }
    if (state.tiltSpeed != MotorSpeed::Off) {
      digitalWrite(TILT_STEP_PIN, LOW);
    }
    delayMicroseconds(maxDelay);
  }
}


MotorsState readMotorsState() {
  byte command;

  // State is a single byte, so get the latest only
  while (Serial.available() > 0) {
    command = (byte)Serial.read();
  }

  MotorDirection panDirection = (MotorDirection)bitRead(command, 7);
  MotorDirection tiltDirection = (MotorDirection)bitRead(command, 6);
  MotorSpeed panSpeed((command & 0b111000) >> 3);
  MotorSpeed tiltSpeed(command & 0b111);

  return MotorsState{
    panDirection,
    tiltDirection,
    panSpeed,
    tiltSpeed
  };
}


void loop() {
  Serial.print("Calibrating Axes");
  calibrateAxes();
  delay(100000);
  // if (Serial.available() > 0) {
  //   char command = (char)Serial.read();
  //   if(command == 'C') {
  //     calibrateAxes();
  //   }
  //   else {
  //   MOTORS_STATE = readMotorsState();
  //   lastCommandRead = millis();
  // }

  // // if it's been more than 0.100 second, then turn off the motors as a safety measure
  // if (millis() - lastCommandRead > 100) {
  //   MOTORS_STATE = MotorsState();
  // }

  // stepMotors(MOTORS_STATE, 1);
  // }
}