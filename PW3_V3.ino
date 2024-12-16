// Include necessary libraries
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <LiquidCrystal.h>

// Create an MPU6050 object
Adafruit_MPU6050 mpu;

// Define motor control pins
int IN1 = A3;
int IN2 = A0;
int IN3 = A2;
int IN4 = A1;
int ENA = 11;
int ENB = 10;

// Line following sensor pins
const int rightIrSensorPin = 13;
const int leftIrSensorPin = 12;

// LCD object for display
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// Variables for tracking the ramp and rotation
float currentRampAngle = 0.0;   // Current ramp angle
float smoothedRampAngle = 0.0;  // Smoothed ramp angle
float smoothingFactor = 0.1;    // Smoothing factor for angle filtering
bool ascending = false;         // Flag for ascending the ramp
bool flatSurfaceDetected = false; // Flag for horizontal surface
bool rotationComplete = false;  // Flag for 360° rotation
bool descending = false;        // Flag for descending the ramp
bool lineFollowing = false;     // Flag for line following after descent
int checkpoint = 1;             // Initial checkpoint
float distanceTravelled = 0.0;  // Distance traveled in cm
unsigned long lastDistanceTime = millis(); // Time tracker for distance calculation
float speedInCmPerSec = 15.0;   // Approximate speed of the robot in cm/s (adjust for your setup)
float maxAngle = 0.0;

// Pin definitions for Encoder
const int encoderPin1 = 3; // Encoder signal pin
volatile int oneCount1 = 0; // Count of '1's for encoder
volatile int previousState1 = LOW; // Previous state of encoderPin1
float distance = 0.0; // Calculated distance
bool carMoving = false;

// Time tracking variables
unsigned long startTime = 0;
unsigned long elapsedTime = 0;
bool timePaused = false;  // Flag to track if the time is paused
unsigned long stopTime = 0; // Time when the robot stopped on black line
unsigned long pausedDuration = 0; // Duration for which the robot was stopped

void setup() {
  // Initialize MPU6050
  Wire.begin();
  Serial.begin(9600);
  lcd.begin(16, 2);
  lcd.print("Angle:");

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // Motor setup
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(encoderPin1, INPUT);

  pinMode(rightIrSensorPin, INPUT);
  pinMode(leftIrSensorPin, INPUT);

  // Update LCD with current values
  lcd.begin(16, 2); // 16x2 LCD
  updateLCD(maxAngle, distance);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);
}

void loop() {
  unsigned long currentTime = millis();  // Continuously running time
  unsigned long elapsedTime = currentTime - startTime - pausedDuration;  // Calculate total elapsed time, excluding paused time

  // Update LCD with the current time (in seconds)
  unsigned long seconds = elapsedTime / 1000;  // Convert milliseconds to seconds

  if (carMoving){
  lcd.setCursor(9, 0);
  lcd.print("T: ");
  lcd.print(seconds);
  lcd.print("s ");
  }

  // MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  //Calculate Angle
  float rawRampAngle = atan2(a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
  smoothedRampAngle = smoothingFactor * rawRampAngle + (1 - smoothingFactor) * smoothedRampAngle;
  currentRampAngle = smoothedRampAngle;
  if (currentRampAngle > maxAngle) {
    maxAngle = currentRampAngle;
  }

  lcd.setCursor(0, 0);
  lcd.print("<:");
  //lcd.print(currentRampAngle - 4.0, 2);
  lcd.print(maxAngle - 6.0);
  lcd.print(" ");

  lcd.setCursor(0,1);
  lcd.print("Dist: ");
  lcd.print(distance, 2);
  lcd.print("m   ");
  
  carMoving = true;

  // Phase 1: Ascending the ramp
if (!ascending && !flatSurfaceDetected && !descending) {
  if (currentRampAngle <= 20) {
    // Move slowly when angle is less than or equal to 5
    moveSlowly();
    Serial.println("Moving slowly...");
  } else if (currentRampAngle > 20) {
    // Transition to ascending once the angle exceeds 5 degrees
    ascending = true;
    motorSpeedUp();
    Serial.println("Ascending the ramp...");
  }
}

  if (ascending) {
    if (currentRampAngle < 10) {
      stopMotors();
      ascending = false;
      flatSurfaceDetected = true;
      Serial.println("Flat surface detected. Stopping...");
      delay(2000);
    }
  }

  // Phase 2: 360° Rotation on flat surface
  if (flatSurfaceDetected && !rotationComplete) {
    rotate360();
    rotationComplete = true;
    Serial.println("360° rotation complete.");
    delay(2000);
    descending = true;
  }

  // Phase 3: Descending the ramp
  if (descending) {
    if (currentRampAngle > 5) {
      moveSlowly(); // Descend at lower speed
      Serial.println("Descending the ramp...");
    }

    if (currentRampAngle < 5) {
      stopMotors();
      delay(2000);
      descending = false;
      lineFollowing = true; // Start line following after descent
      Serial.println("Descent complete. Switching to line following...");
      checkpoint = 0;
      // Attach encoder interrupt
      attachInterrupt(digitalPinToInterrupt(encoderPin1), encoderISR1, CHANGE);
    }
  }

  // Phase 4: Line following
 
  if (lineFollowing) {
    int leftIrValue = digitalRead(leftIrSensorPin);
    int rightIrValue = digitalRead(rightIrSensorPin);

    // Calculate distance
    distance = (oneCount1 * (2 * PI * 3.2)) / 2000.0; // Encoder distance calculation

    // Check if distance is within the range [0.75, 0.8]
    if (distance >= 0.75 && distance <= 0.76) {
      stopMotors();
      delay(2000); // Pause for 2 seconds
      Serial.println("Stopped between 0.75m and 0.8m. Resuming...");
    } else {
      // Continue line-following
      lineFollow(leftIrValue, rightIrValue);
    }
  }

}

// Line following logic
void lineFollow(int leftIrValue, int rightIrValue) {
  if (leftIrValue == 1 && rightIrValue == 1) {  // Both sensors see white surface
    moveForward();
  } 
  else if (leftIrValue == 0 && rightIrValue == 1) {  // Left sees black, right sees white
    pivotRight();
  } 
  else if (rightIrValue == 0 && leftIrValue == 1) {  // Right sees black, left sees white
    pivotLeft();
  } 
  else if (rightIrValue == 0 && leftIrValue == 0) {  // Both sensors detect black
    carMoving = false;
    stopMotors();
  }
  delay(30);  // Small delay to avoid rapid switching
}

// Motor control functions
void moveSlowly() {
  analogWrite(ENA, 85);
  analogWrite(ENB, 80);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void moveForward() {
  analogWrite(ENA, 70);
  analogWrite(ENB, 70);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void motorSpeedUp() {
  analogWrite(ENA, 180);
  analogWrite(ENB, 160);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void pivotLeft() {
  analogWrite(ENA, 225);
  analogWrite(ENB, 225);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void pivotRight() {
  analogWrite(ENA, 225);
  analogWrite(ENB, 225);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void rotate360() {
  float totalRotation = 0.0;
  unsigned long lastTime = millis();
  unsigned long rotationStartTime = millis();
  unsigned long rotationTimeout = 5000;  // Adjusted timeout to be more flexible

  // Start motors to rotate the robot
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 180);  // Motor speed (adjust if needed)
  analogWrite(ENB, 180);  // Motor speed (adjust if needed)

  while (totalRotation < 350.0 && millis() - rotationStartTime < rotationTimeout) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);  // Get gyroscope data

    float gyroZ = g.gyro.z * 180 / PI;  // Convert rad/s to degrees/s
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastTime) / 1000.0;  // Time in seconds
    totalRotation += gyroZ * deltaTime;  // Accumulate total rotation

    lastTime = currentTime;

    // Debugging: Output current rotation progress
    Serial.print("Total Rotation: ");
    Serial.print(totalRotation);
    Serial.println("°");

    // If rotation exceeds 360 degrees, break the loop
    if (totalRotation >= 360.0) {
      break;
    }

    delay(10);  // Short delay to avoid overloading the processor
  }

  // Stop motors after rotation is complete
  stopMotors();
  Serial.println("360-degree rotation complete.");
}

// Interrupt Service Routine (ISR) for Encoder Pin
void encoderISR1() {
  int currentState = digitalRead(encoderPin1);
  if (currentState == HIGH && previousState1 == LOW) {
    oneCount1++;
  }
  previousState1 = currentState;
}

void updateLCD(float maxAngle, float distance) {
  unsigned long elapsedTime = millis() - startTime;  // Calculate elapsed time
  unsigned long seconds = elapsedTime / 1000;         // Convert to seconds

  lcd.setCursor(9,0);
  lcd.print("T: ");
  lcd.print(seconds);
  lcd.print("s ");
  
  lcd.setCursor(0, 0);
  lcd.print("<:");
  lcd.print(maxAngle); // Adjusting angle by subtracting 4.0
  lcd.print(" ");

  lcd.setCursor(9,0);
  lcd.print("T: ");
  lcd.print(seconds);
  lcd.print("s ");

  lcd.setCursor(0, 1);
  lcd.print("Dist: ");
  lcd.print(distance, 2); // Display distance in meters
  lcd.print("m   ");
}


