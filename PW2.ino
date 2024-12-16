#include <LiquidCrystal.h>

// Pin definitions for LCD
const int rs = 8, en = 1, d4 = 4, d5 = 5, d6 = 6, d7 = 7; // LCD pins
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Pin definitions for Encoder
const int encoderPin1 = 2; // Encoder signal pin
volatile int oneCount1 = 0; // Count of '1's for encoder
volatile int previousState1 = LOW; // Previous state of encoderPin1
float distance = 0.0; // Calculated distance

// Pin definitions for IR Sensors and Motors
const int rightIrSensorPin = 13;  // Right IR sensor
const int leftIrSensorPin = 12;   // Left IR sensor
const int IN1 = A3, IN2 = A0, IN3 = A2, IN4 = A1;
const int ENA = 11, ENB = 3;

// Time tracking variables
unsigned long startTime = 0;
unsigned long elapsedTime = 0;
bool carMoving = false;

// Interrupt Service Routine (ISR) for Encoder Pin
void encoderISR1() {
  int currentState = digitalRead(encoderPin1);
  if (currentState == HIGH && previousState1 == LOW) {
    oneCount1++;
  }
  previousState1 = currentState;
}

void setup() {
  // Set pins as inputs or outputs
  pinMode(encoderPin1, INPUT);
  pinMode(rightIrSensorPin, INPUT);
  pinMode(leftIrSensorPin, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Attach encoder interrupt
  attachInterrupt(digitalPinToInterrupt(encoderPin1), encoderISR1, CHANGE);

  // Initialize LCD
  lcd.begin(16, 2);
  lcd.print("Starting...");
  
  // Initialize Serial Monitor
  //Serial.begin(9600);

  // Initial motor speed setup
  analogWrite(ENA, 80);
  analogWrite(ENB, 80);

  delay(700);
  lcd.clear();
}

void loop() {
  int leftIrValue = digitalRead(leftIrSensorPin);
  int rightIrValue = digitalRead(rightIrSensorPin);

  if (leftIrValue == 1 && rightIrValue == 1) {
    moveForward();
  } else if (leftIrValue == 0 && rightIrValue == 1) {
    pivotRight();
  } else if (rightIrValue == 0 && leftIrValue == 1) {
    pivotLeft();
  } else if (rightIrValue == 0 && leftIrValue == 0) {
    stopMotors();
  }

  if (carMoving) {
    elapsedTime = millis() - startTime;
    distance = (oneCount1 * (2 * PI * 3.2)) / 2000.0; // Calculate distance continuously
    displayTime(elapsedTime);
    displayDistance();
  }

  delay(30);
}

void displayDistance() {
  lcd.setCursor(0, 1);
  lcd.print("Dist: ");
  lcd.print(distance, 2);
  lcd.print(" m   ");
}

void displayTime(unsigned long time) {
  unsigned long seconds = time / 1000;
  unsigned long minutes = seconds / 60;
  seconds %= 60;

  lcd.setCursor(0, 0); // First row
  lcd.print("Time: ");
  lcd.print(minutes);
  lcd.print(" m ");
  if (seconds < 10) {
    lcd.print("0"); // Leading zero for single-digit seconds
  }
  lcd.print(seconds);
  lcd.print(" s  "); // Padding to clear old characters
}

void moveForward() {
  analogWrite(ENA, 70);
  analogWrite(ENB, 70);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  if (!carMoving) {
    carMoving = true;
    startTime = millis() - elapsedTime;
  }
}

void pivotLeft() {
  analogWrite(ENA, 215);
  analogWrite(ENB, 215);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void pivotRight() {
  analogWrite(ENA, 215);
  analogWrite(ENB, 215);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
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
  carMoving = false;
}