// Define pins for ultrasonic sensor
const int trigPin = 13;
const int echoPin = 12;

// Define pins for motor driver
const int ena = 3;  // PWM control for motor A
const int in1 = 7;
const int in2 = 8;
const int in3 = 4;
const int in4 = 2;
const int enb = 6;  // PWM control for motor B

// Define desired distance (in cm) to maintain from the wall
const int desiredDistance = 15;
const int tolerance = 5;  // Tolerance of +/- 5 cm

// PID control parameters
double kp = 0.975;  // Proportional gain
double ki = 0.0;  // Integral gain
double kd = 0.33;  // Derivative gain

double prevError = 0;
double integral = 0;

// Ultrasonic sensor timing
const int numReadings = 1;  // Number of readings to average
const int minSensorDelay = 45;  // Minimum delay between measurements in ms

// Define base speed and max speed adjustment
const int baseSpeed = 40;
const int maxSpeed = 60;  // Max speed limit for motors

void setup() {
  // Initialize the ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Initialize the motor driver pins
  pinMode(ena, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enb, OUTPUT);

  // Start serial communication for debugging
  Serial.begin(9600);
}

void loop() {
  // Measure distance to the nearest object
  int distance = getAveragedDistance();

  // Print the distance to the Serial Monitor (for debugging)
  Serial.print("Distance: ");
  Serial.println(distance);

  // Calculate error
  double error = desiredDistance - distance;

  // Separate positive and negative errors
  double positiveError = error > 0 ? error : 0;
  double negativeError = error < 0 ? -error : 0;

  // Calculate PID control variables
  double pidOutputPositive = calculatePID(positiveError);
  double pidOutputNegative = calculatePID(negativeError);

  // Adjust motor speeds based on the PID outputs and turning angles
  adjustMotorSpeeds(pidOutputPositive, pidOutputNegative, error);

  // Small delay to avoid too rapid sensing
  delay(minSensorDelay);
}

int measureDistance() {
  // Send a 10us pulse to trigger pin to start the measurement
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echo pin to get the measurement in microseconds
  long duration = pulseIn(echoPin, HIGH);

  // Calculate the distance (speed of sound is 34300 cm/s)
  int distance = duration * 0.034 / 2;

  return distance;
}

int getAveragedDistance() {
  long totalDistance = 0;

  // Take multiple readings and sum them up
  for (int i = 0; i < numReadings; i++) {
    totalDistance += measureDistance();
    delay(10);  // Short delay between readings
  }

  // Calculate the average distance
  int averageDistance = totalDistance / numReadings;

  return averageDistance;
}

double calculatePID(double error) {
  // Calculate the integral component
  integral += error;

  // Calculate the derivative component
  double derivative = error - prevError;

  // Compute the PID output
  double output = kp * error + ki * integral + kd * derivative;

  // Update the previous error
  prevError = error;

  // Constrain the output to be within the range of motor speed adjustments
  output = constrain(output, 0, 140);  // Limiting the PID output for stability

  return output;
}

void adjustMotorSpeeds(double pidOutputPositive, double pidOutputNegative, double error) {
  // Base speed
  int leftMotorSpeed = baseSpeed;
  int rightMotorSpeed = baseSpeed;

  // Calculate the turning angle based on the error magnitude
  // (You might need to adjust this formula to fit your robot's turning characteristics)
  double turnAdjustment = 0;
  if (error > 0) {
    // Too close to the wall, turn right
    turnAdjustment = pidOutputPositive;
    leftMotorSpeed += turnAdjustment;
    rightMotorSpeed -= turnAdjustment;
  } else if (error < 0) {
    // Too far from the wall, turn left
    turnAdjustment = pidOutputNegative;
    leftMotorSpeed -= turnAdjustment;
    rightMotorSpeed += turnAdjustment;
  }

  // Cap the speeds to the range 0 to maxSpeed
  leftMotorSpeed = constrain(leftMotorSpeed, 0, maxSpeed);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, maxSpeed);

  // Debugging information
  Serial.print("Left Motor Speed: ");
  Serial.print(leftMotorSpeed);
  Serial.print(" Right Motor Speed: ");
  Serial.println(rightMotorSpeed);

  // Move forward with adjusted speeds
  analogWrite(ena, leftMotorSpeed);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enb, rightMotorSpeed);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}