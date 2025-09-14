#include <Arduino.h>
#include <math.h>

// Analog pins
uint8_t speedPin = A0;    // Speed
uint8_t armZPin = A1;     // Right joystick X axis -> Arm Z
uint8_t steeringPin = A2; // Right joystick Y axis -> Steering
uint8_t armXPin = A3;     // Left joystick X axis -> Arm X
uint8_t armYPin = A4;     // Left joystick Y axis -> Arm Y

// Digital button pins
uint8_t gripperPin = 2;             // Right Up Top -> Gripper
uint8_t rightJoystickButtonPin = 3; // Right joystick button -> Stop (momentary)
uint8_t L1Pin = 4;                  // Left Left -> L1
uint8_t R2Pin = 5;                  // Right Down Two -> R2
uint8_t R1Pin = 6;                  // Right Down One -> R1
uint8_t armHomePin = 7;     // Left joystick click -> Arm Home (momentary)
uint8_t R3Pin = 8;          // Right Down Three -> R3
uint8_t autoReleasePin = 9; // Right Up Right -> Auto Release (momentary)
uint8_t leftPin = 10;       // Right Up Left -> Left
uint8_t dumperPin = 11;     // Right Up Down -> Dumper
uint8_t L2Pin = 12;         // Left Right -> L2

// Built-in LED pin
const uint8_t ledPin = LED_BUILTIN;

// Analog variables
int speed = 0;
int steering = 0;
int armX = 0;
int armY = 0;
int armZ = 0;

// Speed limit modes
int speedLimitMode = 0; // 0 = 255, 1 = 50, 2 = 100
int speedLimits[3] = {255, 50, 100};

// Button states
bool gripper = false;
bool stop = false;    // momentary
bool armHome = false; // momentary
bool L1 = false;
bool R1 = false;
bool R2 = false;
bool R3 = false;
bool autoRelease = false; // momentary
bool left = false;
bool dumper = false;
bool L2 = false;

// Previous values for raw inputs
int previousSpeed = 0;
int previousSteering = 0;
int previousArmX = 0;
int previousArmY = 0;
int previousArmZ = 0;

bool previousGripper = false;
bool previousStop = false;
bool previousArmHome = false;
bool previousL1 = false;
bool previousR1 = false;
bool previousR2 = false;
bool previousR3 = false;
bool previousAutoRelease = false;
bool previousLeft = false;
bool previousDumper = false;
bool previousL2 = false;

// Previous values for final output (after L1 logic)
int previousFinalSpeed = 0;
int previousFinalSteering = 0;
int previousFinalArmX = 0;
int previousFinalArmY = 0;
int previousFinalArmZ = 0;
bool previousFinalArmHome = false;
bool previousFinalStop = false;

// Pressed state for toggle buttons
bool gripperPressed = false;
bool L1Pressed = false;
bool R1Pressed = false;
bool R2Pressed = false;
bool R3Pressed = false;
bool autoReleasePressed = false;
bool leftPressed = false;
bool dumperPressed = false;
bool L2Pressed = false;

// Previous pressed state
bool previousGripperButton = false;
bool previousL1Button = false;
bool previousR1Button = false;
bool previousR2Button = false;
bool previousR3Button = false;
bool previousAutoReleaseButton = false;
bool previousLeftButton = false;
bool previousDumperButton = false;
bool previousL2Button = false;

int analog_max = 898;
int analog_min = 134;
int deadzone_min = 510;
int deadzone_max = 520;

int joystick_max = 1023;
int joystick_min = 0;
int joystick_center = 512;

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);

  // Set button pins as INPUT_PULLUP
  pinMode(gripperPin, INPUT_PULLUP);
  pinMode(rightJoystickButtonPin, INPUT_PULLUP); // Stop
  pinMode(L1Pin, INPUT_PULLUP);
  pinMode(R2Pin, INPUT_PULLUP);
  pinMode(R1Pin, INPUT_PULLUP);
  pinMode(armHomePin, INPUT_PULLUP);
  pinMode(R3Pin, INPUT_PULLUP);
  pinMode(autoReleasePin, INPUT_PULLUP);
  pinMode(leftPin, INPUT_PULLUP);
  pinMode(dumperPin, INPUT_PULLUP);
  pinMode(L2Pin, INPUT_PULLUP);

  delay(2000);
}

int applyCurve(int linearValue) {
  float normalized = linearValue / 50.0;
  float curved = pow(normalized, 3); // Cubic curve
  return (int)(curved * 50.0);
}

int mapJoystick(int rawValue, bool invert = false) {
  if (invert)
    rawValue = joystick_max - rawValue + joystick_min;

  if (rawValue > joystick_center) {
    int linearValue = map(rawValue, joystick_center, joystick_max, 0, 50);
    return applyCurve(constrain(linearValue, 0, 50));
  } else if (rawValue < joystick_center) {
    int linearValue = map(rawValue, joystick_min, joystick_center, -50, 0);
    return applyCurve(constrain(linearValue, -50, 0));
  } else
    return 0;
}

void loop() {
  // Read analog inputs
  int rawSpeed = analogRead(speedPin);
  int rawArmZ = analogRead(armZPin);
  int rawSteering = analogRead(steeringPin);
  int rawArmX = analogRead(armXPin);
  int rawArmY = analogRead(armYPin);

  // Read button states
  gripperPressed = !digitalRead(gripperPin);
  bool stopPressed = !digitalRead(rightJoystickButtonPin); // momentary
  armHome = !digitalRead(armHomePin);                      // momentary
  L1Pressed = !digitalRead(L1Pin);
  R1Pressed = !digitalRead(R1Pin);
  R2Pressed = !digitalRead(R2Pin);
  R3Pressed = !digitalRead(R3Pin);
  autoReleasePressed = !digitalRead(autoReleasePin); // momentary
  leftPressed = !digitalRead(leftPin);
  dumperPressed = !digitalRead(dumperPin);
  L2Pressed = !digitalRead(L2Pin);

  // Toggle buttons
  if (gripperPressed && !previousGripperButton)
    gripper = !gripper;
  if (L1Pressed && !previousL1Button) {
    L1 = !L1;
    // Print active mode when L1 changes
    if (L1) {
      Serial.println("Active: Arm");
    } else {
      Serial.println("Active: Car");
    }
  }
  if (R1Pressed && !previousR1Button)
    R1 = !R1;
  if (R2Pressed && !previousR2Button)
    R2 = !R2;
  if (R3Pressed && !previousR3Button)
    R3 = !R3;
  if (leftPressed && !previousLeftButton)
    left = !left;
  if (dumperPressed && !previousDumperButton)
    dumper = !dumper;

  // Handle L2 for speed limit mode cycling with output
  if (L2Pressed && !previousL2Button) {
    speedLimitMode = (speedLimitMode + 1) % 3;

    Serial.print("Mode:");
    switch (speedLimitMode) {
    case 0:
      Serial.println("High");
      break;
    case 1:
      Serial.println("Slow");
      break;
    case 2:
      Serial.println("Medium");
      break;
    }
  }

  // Stop and autoRelease are momentary
  stop = stopPressed;
  autoRelease = autoReleasePressed;

  // Update previous button states
  previousGripperButton = gripperPressed;
  previousL1Button = L1Pressed;
  previousR1Button = R1Pressed;
  previousR2Button = R2Pressed;
  previousR3Button = R3Pressed;
  previousAutoReleaseButton = autoReleasePressed;
  previousLeftButton = leftPressed;
  previousDumperButton = dumperPressed;
  previousL2Button = L2Pressed;

  // Process speed
  int rawSpeedValue = 0;
  if (rawSpeed >= deadzone_max)
    rawSpeedValue = map(rawSpeed, deadzone_max, analog_max, 0, 255);
  else if (rawSpeed <= deadzone_min)
    rawSpeedValue = map(rawSpeed, analog_min, deadzone_min, -255, 0);
  else
    rawSpeedValue = 0;

  rawSpeedValue = constrain(rawSpeedValue, -255, 255);
  if (rawSpeedValue > 0 && rawSpeedValue < 10)
    rawSpeedValue = 0;
  if (rawSpeedValue < 0 && rawSpeedValue > -10)
    rawSpeedValue = 0;

  // Apply speed limit
  int currentSpeedLimit = speedLimits[speedLimitMode];
  if (rawSpeedValue > 0) {
    speed = map(rawSpeedValue, 0, 255, 0, currentSpeedLimit);
  } else if (rawSpeedValue < 0) {
    speed = map(rawSpeedValue, -255, 0, -currentSpeedLimit, 0);
  } else {
    speed = 0;
  }

  // LED on when speed = 0
  digitalWrite(ledPin, speed == 0 ? HIGH : LOW);

  // Process joysticks
  armZ = mapJoystick(rawArmZ);
  steering = mapJoystick(rawSteering);
  armX = mapJoystick(rawArmX);
  armY = mapJoystick(rawArmY, true);

  // Calculate final output values based on L1 state
  int finalSpeed = L1 ? 0 : speed;
  int finalSteering = L1 ? 0 : steering;
  int finalArmX = L1 ? armX : 0;
  int finalArmY = L1 ? armY : 0;
  int finalArmZ = L1 ? armZ : 0;
  bool finalArmHome = L1 ? armHome : false;
  bool finalStop = L1 ? false : stop;

  // Check for significant speed difference (more than 2)
  bool speedChanged = abs(finalSpeed - previousFinalSpeed) > 2;

  // Detect changes in final output values
  bool changed =
      speedChanged || (finalSteering != previousFinalSteering) ||
      (finalArmX != previousFinalArmX) || (finalArmY != previousFinalArmY) ||
      (finalArmZ != previousFinalArmZ) ||
      (finalArmHome != previousFinalArmHome) ||
      (finalStop != previousFinalStop) || (gripper != previousGripper) ||
      (R1 != previousR1) || (R2 != previousR2) || (R3 != previousR3) ||
      (autoRelease != previousAutoRelease) || (left != previousLeft) ||
      (dumper != previousDumper);

  if (changed) {
    Serial.print("speed:");
    Serial.print(finalSpeed);
    Serial.print(",steering:");
    Serial.print(finalSteering);
    Serial.print(",armX:");
    Serial.print(finalArmX);
    Serial.print(",armY:");
    Serial.print(finalArmY);
    Serial.print(",armZ:");
    Serial.print(finalArmZ);
    Serial.print(",armReset:");
    Serial.print(finalArmHome);
    Serial.print(",stop:");
    Serial.print(finalStop);
    Serial.print(",gripper:");
    Serial.print(gripper);
    Serial.print(",autoRelease:");
    Serial.print(autoRelease);
    Serial.print(",left:");
    Serial.print(left);
    Serial.print(",dumper:");
    Serial.print(dumper);
    Serial.print(",R1:");
    Serial.print(R1);
    Serial.print(",R2:");
    Serial.print(R2);
    Serial.print(",R3:");
    Serial.print(R3);
    Serial.println("");

    // Update previous final output values
    previousFinalSpeed = finalSpeed;
    previousFinalSteering = finalSteering;
    previousFinalArmX = finalArmX;
    previousFinalArmY = finalArmY;
    previousFinalArmZ = finalArmZ;
    previousFinalArmHome = finalArmHome;
    previousFinalStop = finalStop;

    // Update other previous values
    previousGripper = gripper;
    previousR1 = R1;
    previousR2 = R2;
    previousR3 = R3;
    previousAutoRelease = autoRelease;
    previousLeft = left;
    previousDumper = dumper;
  }

  delay(50);
}