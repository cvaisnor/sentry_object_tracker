// small test to rotate one stepper motor 180 degrees using the AccelStepper library
#include <AccelStepper.h>

// Define stepper motor connections
#define PAN_STEP_PIN  2
#define PAN_DIR_PIN   5

// Create stepper instances
AccelStepper panStepper(AccelStepper::DRIVER, PAN_STEP_PIN, PAN_DIR_PIN);
 
void setup()
{
  Serial.begin(115200);

  panStepper.setEnablePin(enablePin);
  panStepper.enableOutputs();
  panStepper.setMaxSpeed(1000);
  panStepper.setSpeed(50);
}

void loop()
{
  panStepper.runSpeed();
}