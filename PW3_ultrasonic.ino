#define TRIG_PIN 9
#define ECHO_PIN 8

// Define motor control pins
int IN1 = A3;
int IN2 = A2;
int IN3 = A1;
int IN4 = A0;
int ENA = 10;
int ENB = 11;

bool motorsStopped = false;

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Motor setup
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  Serial.begin(115200);
}

void loop() {
  // Measure distance
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  float distance = duration * 0.034 / 2; // Distance in cm

  Serial.print("Distance: ");
  Serial.println(distance);

    // Check for stop condition
  if (distance <= 15) {
    stopMotors();
    motorsStopped = true;
  } else {
    if (motorsStopped) {
      motorsStopped = false; // Reset the stop flag if safe
    }
    moveSlow(); // Always move slowly unless stopped
  }

// Motor control functions
void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void moveSlow() {
  analogWrite(ENA, 65);
  analogWrite(ENB, 50);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}


