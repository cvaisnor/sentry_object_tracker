// Define endstop pin
#define PAN_LIMIT_PIN  9
#define TILT_LIMIT_PIN 10
void setup() {
  Serial.begin(115200);
  
  // Configure endstop pins
  pinMode(PAN_LIMIT_PIN, INPUT_PULLUP);
  pinMode(TILT_LIMIT_PIN, INPUT_PULLUP);
}
void loop() {
  // small if statement to check if an endstop is triggered
    if (digitalRead(PAN_LIMIT_PIN) == LOW) {
        Serial.println("Pan Endstop triggered!");
    }
    if (digitalRead(TILT_LIMIT_PIN) == LOW) {
        Serial.println("Tilt Endstop triggered!");
    }
}