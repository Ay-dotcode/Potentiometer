#include <Arduino.h>
#include <math.h>

uint8_t potPin = A0; // Acceleration
uint8_t joystickPin = A1;  // Steering
int speed = 0;
int previousSpeed = 0; 
int steering = 0;
int previousSteering = 0;
int joystickRaw = 0;   

// Acceleration parameters
int analog_max = 911;
int analog_min = 90;
int deadzone_min = 510;
int deadzone_max = 520;

// Steering parameters (adjust these based on your joystick's actual range)
int steering_max = 1023;
int steering_min = 0;
int steering_center = 512;

void setup() {
  Serial.begin(115200);
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

  if(rawSpeed >= deadzone_max){
    speed = map(rawSpeed, deadzone_max, analog_max, 0, 255);
  }
  else if(rawSpeed <= deadzone_min){
    speed = map(rawSpeed, analog_min, deadzone_min, -255, 0);
  }
  else{
    speed = 0;
  }
  
  speed = constrain(speed, -255, 255);
  
  if(speed > 0 && speed < 10) {
    speed = 0;
  }
  else if(speed < 0 && speed > -10) {
    speed = 0;
  }

  if(joystickRaw > steering_center) {
    // Right side - map to 0 to 50, then apply curve
    int linearSteering = map(joystickRaw, steering_center, steering_max, 0, 50);
    linearSteering = constrain(linearSteering, 0, 50);
    steering = applyCurve(linearSteering);
  }
  else if(joystickRaw < steering_center) {
    // Left side - map to -50 to 0, then apply curve
    int linearSteering = map(joystickRaw, steering_min, steering_center, -50, 0);
    linearSteering = constrain(linearSteering, -50, 0);
    steering = applyCurve(linearSteering);
  }
  else {
    steering = 0;
  }

  // Send new signal only when speed OR steering changes by more than 1
  bool speedChanged = (speed != previousSpeed && abs(speed - previousSpeed) > 1);
  bool steeringChanged = (steering != previousSteering && abs(steering - previousSteering) > 1);
  
  if(speedChanged || steeringChanged) {
    // Print in format: speed, steering
    Serial.print(speed);
    Serial.print(", ");
    Serial.println(steering);
    previousSpeed = speed;
    previousSteering = steering;
  }
  
  delay(50);
}