#include <Servo.h>

const int trigPin1 = 4;  // Left Sensor
const int echoPin1 = 5;
const int trigPin3 = 2;  // Right Sensor
const int echoPin3 = 3;
const int servoPin = 9;  // Servo Motor

long duration;
int distance1, distance3;
const int limitDistance = 100;  // Distance limit in cm

Servo myServo;
int servoPosition = 90;  // Servo starts at the center position (90 degrees)
bool ledState = LOW;     // Track the current state of the LED

void setup() {
  Serial.begin(9600);

  // Initialize pins for sensors and LED
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);  // Built-in LED

  // Attach servo motor to pin 9
  myServo.attach(servoPin);
  myServo.write(servoPosition);  // Start with the servo in the center position
}

void loop() {
  // Measure distances from left and right sensors
  distance1 = measureDistance(trigPin1, echoPin1);  // Right Sensor
  distance3 = measureDistance(trigPin3, echoPin3);  // Left Sensor

  // Display distances for debugging
  Serial.print("Right: ");
  Serial.print(distance1);
  Serial.print(" cm, Left: ");
  Serial.print(distance3);
  Serial.println(" cm");

  // Servo movement logic
  if (distance3 < limitDistance) {
    // Left sensor detects an object, move servo left
    moveServoLeft();
  } else if (distance1 < limitDistance) {
    // Right sensor detects an object, move servo right
    moveServoRight();
  } else {
    // No movement, turn off LED
    digitalWrite(LED_BUILTIN, LOW);
  }

  delay(20);  // Shorter delay for faster movement
}

int measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.034 / 2;
  return distance;
}

void moveServoLeft() {
  if (servoPosition > 0) {  // Limit servo to 0 degrees (fully left)
    toggleLED();            // Toggle the LED state to create a blinking effect
    servoPosition -= 10;    // Move servo left by 10 degrees at a time for faster speed
    myServo.write(servoPosition);
    Serial.println("Moving Left...");
    delay(50);  // Short delay to create a blinking effect without slowing the servo too much
  }
}

void moveServoRight() {
  if (servoPosition < 180) {  // Limit servo to 180 degrees (fully right)
    toggleLED();              // Toggle the LED state to create a blinking effect
    servoPosition += 10;      // Move servo right by 10 degrees at a time for faster speed
    myServo.write(servoPosition);
    Serial.println("Moving Right...");
    delay(50);  // Short delay to create a blinking effect without slowing the servo too much
  }
}

void toggleLED() {
  ledState = !ledState;                 // Invert the LED state
  digitalWrite(LED_BUILTIN, ledState);  // Set the built-in LED to the new state
}
