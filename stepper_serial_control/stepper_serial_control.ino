byte axis_id;
byte direction;
signed int speed;
signed int distance;

const int panStep = 2;
const int panDir = 5;
const int tiltStep = 3;
const int tiltDir = 6;


void setup() {
  // Initialize your stepper motor driver logic here
  pinMode(panStep, OUTPUT);
  pinMode(panDir, OUTPUT);
  pinMode(tiltStep, OUTPUT);
  pinMode(tiltDir, OUTPUT);
  Serial.begin(9600);
}


void controlMotor(int stepPin, int dirPin, bool direction, int distance, int speed) {
  digitalWrite(dirPin, direction); 
  for (int i = 0; i < distance; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(speed);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(speed);
  }
}


void readMotorCommand() {
  String command = "";
  
  // Check if data is available to read
  if (Serial.available() > 0) {
    // Delay to allow buffer to fill
    delay(20);

    // Read entire string sent from Python
    while (Serial.available() > 0) {
      command += (char)Serial.read();
    }
    // Serial.println(command);    
    
    // Split command into its components
    int axis_id = command.substring(0, command.indexOf(' ')).toInt();
    command = command.substring(command.indexOf(' ')+1);
    int direction = command.substring(0, command.indexOf(' ')).toInt();
    command = command.substring(command.indexOf(' ')+1);
    int speed = command.substring(0, command.indexOf(' ')).toInt();
    command = command.substring(command.indexOf(' ')+1);
    int distance = command.toInt();
    
    // Serial.println(String("Received: Axis = ") + String(axis_id));
    // Serial.println(String("Received: Direction = ") + String(direction));
    // Serial.println(String("Received: Speed = ") + String(speed));
    // Serial.println(String("Received: Distance = ") + String(distance));

    if (axis_id == 0) {
      controlMotor(panStep, panDir, direction, distance, speed);
    } else if (axis_id == 1) {
      controlMotor(tiltStep, tiltDir, direction, distance, speed);
    }
  }
}


void loop() {
  readMotorCommand();  
}