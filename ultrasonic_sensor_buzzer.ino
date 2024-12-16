#define TRIG_PIN 9
#define ECHO_PIN 8
#define BUZZER_PIN 7

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
  pinMode(BUZZER_PIN, OUTPUT);

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
  if (distance <= 5) {
    stopMotors();
    tone(BUZZER_PIN, 1000); // Continuous beep
    motorsStopped = true;
  } else {
    if (motorsStopped) {
      motorsStopped = false; // Reset the stop flag if safe
    }
    moveSlow(); // Always move slowly unless stopped
  }

  // Control actions based on distance
  if (distance > 55) {
    noTone(BUZZER_PIN);
  } else if (distance <= 55 && distance > 35) {
    firstAlarm();
  } else if (distance <= 35 && distance > 5) {
    secondAlarm();
  } else if (distance <= 5) {
    tone(BUZZER_PIN, 1000); // Continuous beep
  }
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

// Alarm functions with non-blocking delay
void firstAlarm() {
  static unsigned long lastTime = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= 500) {
    tone(BUZZER_PIN, 200);
    delay(100);
    noTone(BUZZER_PIN);
    lastTime = currentTime;
  }
}

void secondAlarm() {
  static unsigned long lastTime = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= 200) {
    tone(BUZZER_PIN, 1000);
    delay(40);
    noTone(BUZZER_PIN);
    lastTime = currentTime;
  }
}
