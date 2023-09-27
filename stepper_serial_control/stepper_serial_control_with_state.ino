const int PAN_STEP_PIN = 2;
const int PAN_DIR_PIN = 5;
const int TILT_STEP_PIN = 3;
const int TILT_DIR_PIN = 6;

const int PAN_MIN_PIN = 7;
const int TILT_MIN_PIN = 8;

enum class CalibrationState {
  NotCalibrated,
  Calibrating,
  Calibrated
};

CalibrationState PAN_CALIBRATION_STATE;
CalibrationState TILT_CALIBRATION_STATE;

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


void setup() {
  // Initialize your stepper motor driver logic here
  pinMode(PAN_STEP_PIN, OUTPUT);
  pinMode(PAN_DIR_PIN, OUTPUT);
  pinMode(TILT_STEP_PIN, OUTPUT);
  pinMode(TILT_DIR_PIN, OUTPUT);
  pinMode(PAN_MIN_PIN, INPUT_PULLUP);
  pinMode(TILT_MIN_PIN, INPUT_PULLUP);
  Serial.begin(9600);

  PAN_CALIBRATION_STATE = CalibrationState::NotCalibrated;
  TILT_CALIBRATION_STATE = CalibrationState::NotCalibrated;

  lastCommandRead = millis();
}

void startCalibration() {
  PAN_CALIBRATION_STATE = CalibrationState::Calibrating;
  TILT_CALIBRATION_STATE = CalibrationState::Calibrating;
}

bool isCalibrated() {
  return PAN_CALIBRATION_STATE == CalibrationState::Calibrated && TILT_CALIBRATION_STATE == CalibrationState::Calibrated;
}

void calibrateIfNecessary() {
  if (PAN_CALIBRATION_STATE == CalibrationState::Calibrating && digitalRead(PAN_MIN_PIN) == LOW) {
    PAN_CALIBRATION_STATE = CalibrationState::Calibrated;
    sendSerialMessage("Pan axis calibrated!");
  }

  if (TILT_CALIBRATION_STATE == CalibrationState::Calibrating && digitalRead(TILT_MIN_PIN) == LOW) {
    TILT_CALIBRATION_STATE = CalibrationState::Calibrated;
    sendSerialMessage("Tilt axis calibrated!");
  }
}

void sendSerialMessage(char *msg) {
  Serial.print("*");
  Serial.print(msg);
  Serial.print("#");
}


void stepMotors(MotorsState state, int distance) {
  if (state.panSpeed == MotorSpeed::Off && state.tiltSpeed == MotorSpeed::Off) {
    return;
  }

  digitalWrite(PAN_DIR_PIN, (uint8_t)state.panDirection);
  digitalWrite(TILT_DIR_PIN, (uint8_t)state.tiltDirection);

  // int minDelay = min(
  //   state.panSpeed != MotorSpeed::Off ? state.panSpeed : state.tiltSpeed,
  //   state.tiltSpeed != MotorSpeed::Off ? state.tiltSpeed : state.panSpeed
  // ).getDelay();
  int maxDelay = max(state.panSpeed, state.tiltSpeed).getDelay();

  // TODO: control speed of each direction independantly
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
  if (!isCalibrated()) {
    MOTORS_STATE = MotorsState{ MotorDirection::Down, MotorDirection::Down, MotorSpeed::Speed1, MotorSpeed::Speed1 };
  } else if (Serial.available() > 0) {
    MOTORS_STATE = readMotorsState();
    lastCommandRead = millis();
  }
  else if (PAN_CALIBRATION_STATE == CalibrationState::Calibrating && digitalRead(PAN_MIN_PIN) == LOW) {
    MOTORS_STATE.panSpeed = MotorSpeed::Off;
  } else if (TILT_CALIBRATION_STATE == CalibrationState::Calibrating && digitalRead(TILT_MIN_PIN) == LOW) {
    MOTORS_STATE.tiltSpeed = MotorSpeed::Off;
  }
  
  // if it's been more than # second, then turn off the motors as a safety measure
  if (millis() - lastCommandRead > 100) {
    MOTORS_STATE = MotorsState();
  }

  stepMotors(MOTORS_STATE, 1);
}
