#include <Arduino.h>
#include <math.h>

uint8_t potPin = A0;            // Acceleration
uint8_t joystickPin = A1;       // Steering
uint8_t stopButtonPin = 2;      // Stop button
uint8_t gripperButtonPin = 3;   // Open gripper button
uint8_t dumptruckButtonPin = 4; // Open dumptruck button

int speed = 0;
int previousSpeed = 0;
int steering = 0;
int previousSteering = 0;
int joystickRaw = 0;

bool stop = false;
bool previousStop = false;
bool gripper = false;
bool dumptruck = false;
bool previousGripper = false;
bool previousDumptruck = false;

// Button state tracking for toggle functionality
bool gripperButtonPressed = false;
bool dumptruckButtonPressed = false;
bool previousGripperButton = false;
bool previousDumptruckButton = false;

// Acceleration parameters
int analog_max = 916;
int analog_min = 133;
int deadzone_min = 510;
int deadzone_max = 520;

// Steering parameters (adjust these based on your joystick's actual range)
int steering_max = 1023;
int steering_min = 0;
int steering_center = 512;

void setup() {
  Serial.begin(115200);

  // Configure all button pins with internal pull-up resistors
  pinMode(stopButtonPin, INPUT_PULLUP);
  pinMode(gripperButtonPin, INPUT_PULLUP);
  pinMode(dumptruckButtonPin, INPUT_PULLUP);

  delay(2000);
}

int applyCurve(int linearValue) {
  // Convert to float for calculation (-1.0 to 1.0)
  float normalized = linearValue / 50.0;
  float curved = pow(normalized, 3); // Cubic curve

  return (int)(curved * 50.0);
}

void loop() {
  int rawSpeed = analogRead(potPin);
  joystickRaw = analogRead(joystickPin);

  // Read all button states (inverted because of pull-up resistors)
  stop = !digitalRead(stopButtonPin);
  gripperButtonPressed = !digitalRead(gripperButtonPin);
  dumptruckButtonPressed = !digitalRead(dumptruckButtonPin);

  // Toggle gripper when button is pressed (rising edge detection)
  if (gripperButtonPressed && !previousGripperButton) {
    gripper = !gripper; // Toggle the gripper state
  }

  // Toggle dumptruck when button is pressed (rising edge detection)
  if (dumptruckButtonPressed && !previousDumptruckButton) {
    dumptruck = !dumptruck; // Toggle the dumptruck state
  }

  // Update previous button states
  previousGripperButton = gripperButtonPressed;
  previousDumptruckButton = dumptruckButtonPressed;

  if (rawSpeed >= deadzone_max) {
    speed = map(rawSpeed, deadzone_max, analog_max, 0, 255);
  } else if (rawSpeed <= deadzone_min) {
    speed = map(rawSpeed, analog_min, deadzone_min, -255, 0);
  } else {
    speed = 0;
  }

  speed = constrain(speed, -255, 255);

  if (speed > 0 && speed < 10) {
    speed = 0;
  } else if (speed < 0 && speed > -10) {
    speed = 0;
  }

  if (joystickRaw > steering_center) {
    // Right side - map to 0 to 50, then apply curve
    int linearSteering = map(joystickRaw, steering_center, steering_max, 0, 50);
    linearSteering = constrain(linearSteering, 0, 50);
    steering = applyCurve(linearSteering);
  } else if (joystickRaw < steering_center) {
    // Left side - map to -50 to 0, then apply curve
    int linearSteering =
        map(joystickRaw, steering_min, steering_center, -50, 0);
    linearSteering = constrain(linearSteering, -50, 0);
    steering = applyCurve(linearSteering);
  } else {
    steering = 0;
  }

  // Check for changes in any values
  bool speedChanged =
      (speed != previousSpeed && abs(speed - previousSpeed) > 1);
  bool steeringChanged =
      (steering != previousSteering && abs(steering - previousSteering) > 1);
  bool stopChanged = (stop != previousStop);
  bool gripperChanged = (gripper != previousGripper);
  bool dumptruckChanged = (dumptruck != previousDumptruck);

  if (speedChanged || steeringChanged || stopChanged || gripperChanged ||
      dumptruckChanged) {

    // Print in key-value pair format
    Serial.print("{speed:");
    Serial.print(speed);
    Serial.print(",steering:");
    Serial.print(steering);
    Serial.print(",stop:");
    Serial.print(stop);
    Serial.print(",gripper:");
    Serial.print(gripper);
    Serial.print(",dumptruck:");
    Serial.print(dumptruck);
    Serial.println("}");

    // Update previous values
    previousSpeed = speed;
    previousSteering = steering;
    previousStop = stop;
    previousGripper = gripper;
    previousDumptruck = dumptruck;
  }

  delay(50);
}