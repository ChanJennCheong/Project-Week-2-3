#include <Servo.h>          //Servo motor library. This is standard library //Robot Lk
#include <NewPing.h>        //Ultrasonic sensor function library. You must install this library //Robot Lk

// Define motor control pins
int IN1 = A3;
int IN2 = A2;
int IN3 = A1;
int IN4 = A0;
int ENA = 3;
int ENB = 11;
int LR = 4;
int RR = 5;

int BUZZER_PIN = 7;

//sensor pins
#define trig_pin 9
#define echo_pin 8
#define maximum_distance 200
boolean goesForward = false;
int distance = 100;
NewPing sonar(trig_pin, echo_pin, maximum_distance); //sensor function
Servo servo_motor; //our servo name

void setup(){
  // Motor setup
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LR, OUTPUT);
  pinMode(RR, OUTPUT);

  servo_motor.attach(6); //our servo pin
  servo_motor.write(115);

  delay(2000);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
}

void loop() {
  int distanceRight = 0;
  int distanceLeft = 0;
  delay(50);

  if (distance <= 45) {
    moveStop();
    digitalWrite(BUZZER_PIN, HIGH); // Turn on the buzzer
    delay(300);                     // Duration
    digitalWrite(BUZZER_PIN, LOW);  // Turn off the buzzer
    delay(300);
    moveBackward();
    delay(400);
    moveStop();
    delay(300);

    distanceRight = lookRight();
    delay(300);
    distanceLeft = lookLeft();
    delay(300);

    // Updated decision-making logic
    if (distanceRight == 0 && distanceLeft == 0) {
      // Both sides are free; default to turning right
      turnLeft();
    } else if (distanceRight > distanceLeft) {
      // Turn to the side with more space
      turnRight();
    } else if (distanceLeft > distanceRight) {
      turnLeft();
    }
    moveStop();
  } else {
    moveForward();
  }

  // Update the distance reading
  distance = readPing();
}


int lookRight(){
  servo_motor.write(50);
  delay(500);
  int distance = readPing();
  delay(100);
  servo_motor.write(115);
  return distance;
}

int lookLeft(){
  servo_motor.write(170);
  delay(500);
  int distance = readPing();
  delay(100);
  servo_motor.write(115);
  return distance;
  delay(100);
}

int readPing(){
  delay(70);
  int cm = sonar.ping_cm();
  if (cm==0){
    cm=250;
  }
  return cm;
}

void moveStop(){
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void moveForward(){
  if(!goesForward){
    goesForward=true;
    analogWrite(ENA, 90);
    analogWrite(ENB, 85);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    digitalWrite(LR, LOW);
    digitalWrite(RR, LOW);
  }
}

void moveBackward(){
  goesForward=false;
    analogWrite(ENA, 150);
    analogWrite(ENB, 145);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}
void turnLeft(){
  analogWrite(ENA, 150);
  analogWrite(ENB, 145);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  digitalWrite(LR, HIGH);
  digitalWrite(RR, LOW);
  delay(250);
}

void turnRight(){
  analogWrite(ENA, 150);
  analogWrite(ENB, 145);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  digitalWrite(RR, HIGH);
  digitalWrite(LR, LOW);
  delay(250);
}
