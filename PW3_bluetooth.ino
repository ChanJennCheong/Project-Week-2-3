#define FORWARD 'F'
#define BACKWARD 'B'
#define LEFT 'L'
#define RIGHT 'R'
#define CROSS 'X'

#include <Arduino.h>
#include <LiquidCrystal.h>
#include <PinChangeInterrupt.h>

// Motor Pins
int IN1 = A3;
int IN2 = A2;
int IN3 = A1;
int IN4 = A0;
int FNA = 10;
int FNB = 11;

void setup() {
  Serial.begin(9600);  // Set the baud rate for serial communication

  // Set the output pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(FNA, OUTPUT);
  pinMode(FNB, OUTPUT);
 
}

void setMotorSpeed(int motorPWM, int speed) {
  speed = constrain(speed, 0, 255);
  analogWrite(motorPWM, speed);
}

void MoveForward(int PWM) {
  setMotorSpeed(FNA, PWM);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  setMotorSpeed(FNB, PWM);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void MoveBackward(){
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4,HIGH);

}

void Stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void Left() {
  setMotorSpeed(FNA, 255);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  setMotorSpeed(FNB, 255);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void Right() {
  setMotorSpeed(FNA, 255);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  setMotorSpeed(FNB, 255);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void executeCommand(char command) {
  switch (command) {
    case FORWARD:
      MoveForward(255);  // Move forward at full speed
      break;
    case BACKWARD:
      MoveBackward(); // Perform action for moving backward (you need to define this function)
      break;
    case LEFT:
      Right(); // Turn left
      break;
    case RIGHT:
      Left();  // Turn right
      break;
    case CROSS:
      Stop();  // Stop immediately
      break;
  }
}

void loop() {
  if (Serial.available()) {
    char command = Serial.read();
    executeCommand(command);  // Execute the appropriate command based on serial input
  }

  
  


















  







  
}