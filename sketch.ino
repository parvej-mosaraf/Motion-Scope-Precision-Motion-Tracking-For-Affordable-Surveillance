#include <Servo.h>
// #include <PID_v1.h>  aita gpt bole but khuija pai nai
#include <PID_v1_bc.h>
#include <SimpleKalmanFilter.h>

// Ultrasonic sensor pins
const int trigPin1 = 3;  // Left Sensor
const int echoPin1 = 4;
const int trigPin2 = 5;  // Center Sensor
const int echoPin2 = 6;
const int trigPin3 = 7;  // Right Sensor
const int echoPin3 = 8;

// PIR sensor pin
const int pirPin = 10;   // Motion Detection

// Servo motor pin
const int servoPin = 9;

// Kalman Filter for each sensor
SimpleKalmanFilter kalman1(1, 1, 0.01);
SimpleKalmanFilter kalman2(1, 1, 0.01);
SimpleKalmanFilter kalman3(1, 1, 0.01);

Servo tracerServo;

double Setpoint, Input, Output;
// double Kp = 1, Ki = 0.05, Kd = 0.25;
double Kp = 0.1, Ki = 0.01, Kd = 0.01;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Constants for smoothing motion
const int smoothFactor = 5;
const int minPos = 0;
const int maxPos = 180;
int currentPos = 90; // Start at the center

void setup() {
  Serial.begin(9600);

  // Servo setup
  tracerServo.attach(servoPin);
  tracerServo.write(currentPos);

  // PID setup
  Setpoint = 90; // Desired centered position
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-15, 15); // Limit servo movement speed
  
  // Pin mode setup for ultrasonic sensors
  pinMode(trigPin1, OUTPUT); // Left Sensor
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT); // Center Sensor
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT); // Right Sensor
  pinMode(echoPin3, INPUT);

  // Pin mode setup for PIR sensor
  pinMode(pirPin, INPUT); // Motion Detection
}

// Define a distance threshold
const double distanceThreshold = 18.0;  // 50 cm

void loop() {
  // Check PIR sensor for motion detection
  // bool motionDetected = digitalRead(pirPin) == HIGH;
   bool motionDetected = true;

  if (motionDetected) {
    double distance1 = getFilteredDistance(trigPin1, echoPin1, kalman1); // Left Sensor
    double distance2 = getFilteredDistance(trigPin2, echoPin2, kalman2); // Center Sensor
    double distance3 = getFilteredDistance(trigPin3, echoPin3, kalman3); // Right Sensor

    // Check if any of the sensors detect an object within the threshold distance
    if (distance1 < distanceThreshold || distance2 < distanceThreshold || distance3 < distanceThreshold) {
      double weightedPosition = calculateWeightedPosition(distance1, distance2, distance3);
      double predictedPosition = predictPosition(weightedPosition);
       Serial.println(predictedPosition);


      Input = predictedPosition;
      myPID.Compute();

      currentPos = constrain(currentPos + Output, minPos, maxPos);
      tracerServo.write(currentPos);
    }
  } else {
    // Optionally, stop the servo or set it to a default position
    tracerServo.write(90);  // Center position or any other default
  }
  delay(50);  // Short delay for smoother motion
}


double getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  double distance = (duration * 0.034) / 2; // cm
  return distance;
}

double getFilteredDistance(int trigPin, int echoPin, SimpleKalmanFilter &kalmanFilter) {
  double rawDistance = getDistance(trigPin, echoPin);
  return kalmanFilter.updateEstimate(rawDistance);
}

double calculateWeightedPosition(double d1, double d2, double d3) {
  double position = (d1 * -45 + d2 * 0 + d3 * 45) / (d1 + d2 + d3);
  return map(position, -45, 45, minPos, maxPos);
}

double predictPosition(double currentPos) {
  static double lastPos = 90;
  double predictedPos = currentPos + (currentPos - lastPos) * smoothFactor;
  lastPos = currentPos;
  return predictedPos;
}
