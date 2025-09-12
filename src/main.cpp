#include <Arduino.h>
#include <math.h>

// Analog pins
uint8_t movementPin = A0;       // Movement
uint8_t rightJoystickXPin = A1; // Right joystick X axis
uint8_t rightJoystickYPin = A2; // Right joystick Y axis
uint8_t leftJoystickXPin = A3;  // Left joystick X axis
uint8_t leftJoystickYPin = A4;  // Left joystick Y axis

// Digital button pins
uint8_t rightUpTopPin = 2;      // Right up top button
uint8_t rightJoystickPin = 3;   // Right joystick button
uint8_t leftLeftPin = 4;        // Left left button
uint8_t rightDownTwoPin = 5;    // Right down two button
uint8_t rightDownOnePin = 6;    // Right down one button
uint8_t leftJoystickPin = 7;    // Left joystick button
uint8_t rightDownThreePin = 8;  // Right down three button
uint8_t rightTopRightPin = 9;   // Right top right button
uint8_t rightTopLeftPin = 10;   // Right top left button
uint8_t rightTopButtonPin = 11; // Right top button
uint8_t leftRightPin = 12;      // Left right button

// Movement and control variables
int movement = 0;
int rightJoystickX = 0;
int rightJoystickY = 0;
int leftJoystickX = 0;
int leftJoystickY = 0;

// Button states
bool rightUpTop = false;
bool rightJoystickButton = false;
bool leftLeft = false;
bool rightDownTwo = false;
bool rightDownOne = false;
bool leftJoystickButton = false;
bool rightDownThree = false;
bool rightTopRight = false;
bool rightTopLeft = false;
bool rightTopButton = false;
bool leftRight = false;

// Previous values for change detection
int previousMovement = 0;
int previousRightJoystickX = 0;
int previousRightJoystickY = 0;
int previousLeftJoystickX = 0;
int previousLeftJoystickY = 0;

bool previousRightUpTop = false;
bool previousRightJoystickButton = false;
bool previousLeftLeft = false;
bool previousRightDownTwo = false;
bool previousRightDownOne = false;
bool previousLeftJoystickButton = false;
bool previousRightDownThree = false;
bool previousRightTopRight = false;
bool previousRightTopLeft = false;
bool previousRightTopButton = false;
bool previousLeftRight = false;

// Button state tracking for toggle functionality
bool rightUpTopPressed = false;
bool rightJoystickButtonPressed = false;
bool leftLeftPressed = false;
bool rightDownTwoPressed = false;
bool rightDownOnePressed = false;
bool leftJoystickButtonPressed = false;
bool rightDownThreePressed = false;
bool rightTopRightPressed = false;
bool rightTopLeftPressed = false;
bool rightTopButtonPressed = false;
bool leftRightPressed = false;

bool previousRightUpTopButton = false;
bool previousRightJoystickButtonButton = false;
bool previousLeftLeftButton = false;
bool previousRightDownTwoButton = false;
bool previousRightDownOneButton = false;
bool previousLeftJoystickButtonButton = false;
bool previousRightDownThreeButton = false;
bool previousRightTopRightButton = false;
bool previousRightTopLeftButton = false;
bool previousRightTopButtonButton = false;
bool previousLeftRightButton = false;

// Movement parameters
int analog_max = 922;
int analog_min = 155;
int deadzone_min = 510;
int deadzone_max = 520;

// Joystick parameters
int joystick_max = 1023;
int joystick_min = 0;
int joystick_center = 512;

void setup() {
  Serial.begin(115200);

  // Configure all button pins with internal pull-up resistors
  pinMode(rightUpTopPin, INPUT_PULLUP);
  pinMode(rightJoystickPin, INPUT_PULLUP);
  pinMode(leftLeftPin, INPUT_PULLUP);
  pinMode(rightDownTwoPin, INPUT_PULLUP);
  pinMode(rightDownOnePin, INPUT_PULLUP);
  pinMode(leftJoystickPin, INPUT_PULLUP);
  pinMode(rightDownThreePin, INPUT_PULLUP);
  pinMode(rightTopRightPin, INPUT_PULLUP);
  pinMode(rightTopLeftPin, INPUT_PULLUP);
  pinMode(rightTopButtonPin, INPUT_PULLUP);
  pinMode(leftRightPin, INPUT_PULLUP);

  delay(2000);
}

int applyCurve(int linearValue) {
  // Convert to float for calculation (-1.0 to 1.0)
  float normalized = linearValue / 50.0;
  float curved = pow(normalized, 3); // Cubic curve

  return (int)(curved * 50.0);
}

int mapJoystick(int rawValue, bool invert = false) {
  if (invert) {
    // Invert the raw value for upside-down joystick
    rawValue = joystick_max - rawValue + joystick_min;
  }

  if (rawValue > joystick_center) {
    // Right/Up side - map to 0 to 50
    int linearValue = map(rawValue, joystick_center, joystick_max, 0, 50);
    linearValue = constrain(linearValue, 0, 50);
    return applyCurve(linearValue);
  } else if (rawValue < joystick_center) {
    // Left/Down side - map to -50 to 0
    int linearValue = map(rawValue, joystick_min, joystick_center, -50, 0);
    linearValue = constrain(linearValue, -50, 0);
    return applyCurve(linearValue);
  } else {
    return 0;
  }
}

void loop() {
  // Read all analog inputs
  int rawMovement = analogRead(movementPin);
  int rawRightJoystickX = analogRead(rightJoystickXPin);
  int rawRightJoystickY = analogRead(rightJoystickYPin);
  int rawLeftJoystickX = analogRead(leftJoystickXPin);
  int rawLeftJoystickY = analogRead(leftJoystickYPin);

  // Read all button states (inverted because of pull-up resistors)
  rightUpTopPressed = !digitalRead(rightUpTopPin);
  rightJoystickButtonPressed = !digitalRead(rightJoystickPin);
  leftLeftPressed = !digitalRead(leftLeftPin);
  rightDownTwoPressed = !digitalRead(rightDownTwoPin);
  rightDownOnePressed = !digitalRead(rightDownOnePin);
  leftJoystickButtonPressed = !digitalRead(leftJoystickPin);
  rightDownThreePressed = !digitalRead(rightDownThreePin);
  rightTopRightPressed = !digitalRead(rightTopRightPin);
  rightTopLeftPressed = !digitalRead(rightTopLeftPin);
  rightTopButtonPressed = !digitalRead(rightTopButtonPin);
  leftRightPressed = !digitalRead(leftRightPin);

  // Toggle buttons when pressed (rising edge detection)
  if (rightUpTopPressed && !previousRightUpTopButton) {
    rightUpTop = !rightUpTop;
  }
  if (rightJoystickButtonPressed && !previousRightJoystickButtonButton) {
    rightJoystickButton = !rightJoystickButton;
  }
  if (leftLeftPressed && !previousLeftLeftButton) {
    leftLeft = !leftLeft;
  }
  if (rightDownTwoPressed && !previousRightDownTwoButton) {
    rightDownTwo = !rightDownTwo;
  }
  if (rightDownOnePressed && !previousRightDownOneButton) {
    rightDownOne = !rightDownOne;
  }
  if (leftJoystickButtonPressed && !previousLeftJoystickButtonButton) {
    leftJoystickButton = !leftJoystickButton;
  }
  if (rightDownThreePressed && !previousRightDownThreeButton) {
    rightDownThree = !rightDownThree;
  }
  if (rightTopRightPressed && !previousRightTopRightButton) {
    rightTopRight = !rightTopRight;
  }
  if (rightTopLeftPressed && !previousRightTopLeftButton) {
    rightTopLeft = !rightTopLeft;
  }
  if (rightTopButtonPressed && !previousRightTopButtonButton) {
    rightTopButton = !rightTopButton;
  }
  if (leftRightPressed && !previousLeftRightButton) {
    leftRight = !leftRight;
  }

  // Update previous button states
  previousRightUpTopButton = rightUpTopPressed;
  previousRightJoystickButtonButton = rightJoystickButtonPressed;
  previousLeftLeftButton = leftLeftPressed;
  previousRightDownTwoButton = rightDownTwoPressed;
  previousRightDownOneButton = rightDownOnePressed;
  previousLeftJoystickButtonButton = leftJoystickButtonPressed;
  previousRightDownThreeButton = rightDownThreePressed;
  previousRightTopRightButton = rightTopRightPressed;
  previousRightTopLeftButton = rightTopLeftPressed;
  previousRightTopButtonButton = rightTopButtonPressed;
  previousLeftRightButton = leftRightPressed;

  // Process movement with deadzone
  if (rawMovement >= deadzone_max) {
    movement = map(rawMovement, deadzone_max, analog_max, 0, 255);
  } else if (rawMovement <= deadzone_min) {
    movement = map(rawMovement, analog_min, deadzone_min, -255, 0);
  } else {
    movement = 0;
  }

  movement = constrain(movement, -255, 255);

  // Apply minimum threshold to movement
  if (movement > 0 && movement < 10) {
    movement = 0;
  } else if (movement < 0 && movement > -10) {
    movement = 0;
  }

  // Process joystick inputs (left joystick Y is inverted)
  rightJoystickX = mapJoystick(rawRightJoystickX);
  rightJoystickY = mapJoystick(rawRightJoystickY);
  leftJoystickX = mapJoystick(rawLeftJoystickX);
  leftJoystickY = mapJoystick(rawLeftJoystickY, true); // Invert left joystick Y

  // Check for changes in any values
  bool movementChanged =
      (movement != previousMovement && abs(movement - previousMovement) > 1);
  bool rightJoystickXChanged =
      (rightJoystickX != previousRightJoystickX &&
       abs(rightJoystickX - previousRightJoystickX) > 1);
  bool rightJoystickYChanged =
      (rightJoystickY != previousRightJoystickY &&
       abs(rightJoystickY - previousRightJoystickY) > 1);
  bool leftJoystickXChanged = (leftJoystickX != previousLeftJoystickX &&
                               abs(leftJoystickX - previousLeftJoystickX) > 1);
  bool leftJoystickYChanged = (leftJoystickY != previousLeftJoystickY &&
                               abs(leftJoystickY - previousLeftJoystickY) > 1);

  bool rightUpTopChanged = (rightUpTop != previousRightUpTop);
  bool rightJoystickButtonChanged =
      (rightJoystickButton != previousRightJoystickButton);
  bool leftLeftChanged = (leftLeft != previousLeftLeft);
  bool rightDownTwoChanged = (rightDownTwo != previousRightDownTwo);
  bool rightDownOneChanged = (rightDownOne != previousRightDownOne);
  bool leftJoystickButtonChanged =
      (leftJoystickButton != previousLeftJoystickButton);
  bool rightDownThreeChanged = (rightDownThree != previousRightDownThree);
  bool rightTopRightChanged = (rightTopRight != previousRightTopRight);
  bool rightTopLeftChanged = (rightTopLeft != previousRightTopLeft);
  bool rightTopButtonChanged = (rightTopButton != previousRightTopButton);
  bool leftRightChanged = (leftRight != previousLeftRight);

  // Print all values if any have changed
  if (movementChanged || rightJoystickXChanged || rightJoystickYChanged ||
      leftJoystickXChanged || leftJoystickYChanged || rightUpTopChanged ||
      rightJoystickButtonChanged || leftLeftChanged || rightDownTwoChanged ||
      rightDownOneChanged || leftJoystickButtonChanged ||
      rightDownThreeChanged || rightTopRightChanged || rightTopLeftChanged ||
      rightTopButtonChanged || leftRightChanged) {

    // Print in key-value pair format
    Serial.print("movement:");
    Serial.print(movement);
    // Serial.print("raw:");
    // Serial.print(rawMovement);

    Serial.print(",Rjoystick:");
    Serial.print(rightJoystickX);
    Serial.print(",");
    Serial.print(rightJoystickY);
    Serial.print(",Rbutton:");
    Serial.print(rightJoystickButton);

    Serial.print(",Ljoystick:");
    Serial.print(leftJoystickX);
    Serial.print(",");
    Serial.print(leftJoystickY);
    Serial.print(",Lbutton:");
    Serial.print(leftJoystickButton);

    Serial.print(",LeftLeft:");
    Serial.print(leftLeft);
    Serial.print(",LeftRight:");
    Serial.print(leftRight);

    Serial.print(",RDownOne:");
    Serial.print(rightDownOne);
    Serial.print(",RDownTwo:");
    Serial.print(rightDownTwo);
    Serial.print(",RDownThree:");
    Serial.print(rightDownThree);
    
    Serial.print(",RightUpTop:");
    Serial.print(rightUpTop);
    Serial.print(",RTopRight:");
    Serial.print(rightTopRight);
    Serial.print(",RTopLeft:");
    Serial.print(rightTopLeft);
    Serial.print(",RTopButton:");
    Serial.println(rightTopButton);

    // Update previous values
    previousMovement = movement;
    previousRightJoystickX = rightJoystickX;
    previousRightJoystickY = rightJoystickY;
    previousLeftJoystickX = leftJoystickX;
    previousLeftJoystickY = leftJoystickY;
    previousRightUpTop = rightUpTop;
    previousRightJoystickButton = rightJoystickButton;
    previousLeftLeft = leftLeft;
    previousRightDownTwo = rightDownTwo;
    previousRightDownOne = rightDownOne;
    previousLeftJoystickButton = leftJoystickButton;
    previousRightDownThree = rightDownThree;
    previousRightTopRight = rightTopRight;
    previousRightTopLeft = rightTopLeft;
    previousRightTopButton = rightTopButton;
    previousLeftRight = leftRight;
  }

  delay(50);
}