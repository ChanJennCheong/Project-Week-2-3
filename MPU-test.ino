#include "Adafruit_MPU6050.h"
#include "Adafruit_Sensor.h"
#include "Wire.h"

// Motor driver pins
#define IN1 A3
#define IN2 10
#define IN3 A2
#define IN4 A1
const int ENA = 11, ENB = 3;

Adafruit_MPU6050 mpu;

// Variables for tracking the ramp and rotation
float maxRampAngle = 0.0;       // Maximum angle reached during ramp climb
float currentRampAngle = 0.0;   // Current ramp angle
float smoothedRampAngle = 0.0;  // Smoothed ramp angle (for stability)
float smoothingFactor = 0.1;    // Smoothing factor for angle filtering
bool atPeak = false;            // Flag to indicate if the peak has been reached
bool descentStarted = false;    // Flag to track descent
unsigned long peakStartTime = 0; // Time when peak was reached
unsigned long peakStableTime = 2000; // Time in milliseconds to wait before starting the turn

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("Ramp Angle Measurement with MPU6050");

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  Serial.println("MPU6050 initialized!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float rawRampAngle = atan2(a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;
  smoothedRampAngle = smoothingFactor * rawRampAngle + (1 - smoothingFactor) * smoothedRampAngle;
  currentRampAngle = smoothedRampAngle;

  Serial.print("Ramp Angle: ");
  Serial.print(currentRampAngle);
  Serial.println("°");

  // Ramp climbing logic
  if (!atPeak && !descentStarted) {
    if (currentRampAngle > 5) {
      climbRamp();
      maxRampAngle = max(maxRampAngle, currentRampAngle);
    } else {
      moveForward();
    }

    if (currentRampAngle < maxRampAngle - 5 && peakStartTime == 0) {
      peakStartTime = millis();
      Serial.println("Peak detected! Waiting for stabilization...");
    }

    if (peakStartTime > 0 && millis() - peakStartTime > peakStableTime) {
      atPeak = true;
      stopMotors();
      Serial.println("Peak stabilized, starting 360-degree rotation...");
      rotate360();
    }
  }

  // Descent logic
  if (atPeak && !descentStarted) {
    descentStarted = true;
    Serial.println("Starting descent...");
  }

  if (descentStarted) {
    if (currentRampAngle > -5) {
      moveForward();
    } else {
      stopMotors();
      Serial.println("Descent complete.");
      while (1); // Stop program
    }
  }
}

void moveForward() {
  analogWrite(ENA, 80);
  analogWrite(ENB, 80);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void climbRamp() {
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void rotate360() {
  float totalRotation = 0.0;
  unsigned long lastTime = millis();
  unsigned long rotationStartTime = millis();
  unsigned long rotationTimeout = 4500;

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 150);
  analogWrite(ENB, 150);

  while (totalRotation < 340.0 && millis() - rotationStartTime < rotationTimeout) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float gyroZ = g.gyro.z * 180 / PI;
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastTime) / 1000.0;
    totalRotation += gyroZ * deltaTime;
    lastTime = currentTime;

    Serial.print("Total Rotation: ");
    Serial.print(totalRotation);
    Serial.println("°");

    delay(10);
  }

  stopMotors();
  Serial.println("360-degree rotation complete.");
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  delay(500);
}
