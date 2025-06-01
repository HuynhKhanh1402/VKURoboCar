#define IN1 2
#define IN2 4
#define ENA 3
#define IN3 5
#define IN4 7
#define ENB 6

#define TRIG_PIN 8
#define ECHO_PIN 9

const uint8_t SENSORS_PIN[] = { A0, A1, A2, A3, A4 };

const int BASE_SPEED = 80;  // Nominal forward speed (0-255)
const float Kp = 55;         // Proportional gain
// const float Ki = 0.02;             // Integral gain
const float Ki = 0.02;  // Integral gain
// const float Kd = 0.3;              // Derivative gain
const float Kd = 0.3;                // Derivative gain
const float INTEGRAL_MAX = 1000;     // Anti-windup cap for integral term
const float D_FILTER_TAU = 0.02;     // Derivative filter time constant (s)
const int CONTROL_INTERVAL_MS = 10;  // Control loop period

const int MAX_SPEED = 120;
const int REVERSE_MAX_SPEED = -120;
const int STRAIGHT_SPEED = 100;

// Error mapping weights for 5 sensors: positions -2, -1, 0, 1, 2
const int weights[5] = { -2, -1, 0, 1, 2 };

uint16_t sensor_max[] = { 300, 300, 300, 300, 300 };
uint16_t sensor_min[] = { 30, 30, 30, 30, 30 };
uint16_t sensors[] = { -1, -1, -1, -1, -1 };
String sensorArray = "";

float distance = 999999;

// PID state
float integralTerm = 0;
float prevError = 0;
float prevDerivative = 0;
unsigned long lastTime = 0;

int lastErrorDirection = 0;

static bool isClimbing = false;
static int stairClimbAttempts = 0;
static unsigned long climbStartTime = 0;

const int APPROACH_SPEED = 80;
const unsigned long APPROACH_TIME = 300UL;
const unsigned long PAUSE_BEFORE = 200UL;

bool isPaused = false;
unsigned long detectTime = 0;
bool waitingConfirmation = false;


void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  for (auto sensorPin : SENSORS_PIN) {
    pinMode(sensorPin, INPUT);
  }
  pinMode(A6, INPUT);
  Serial.begin(115200);
}

void loop() {
  delay(2000);
  setMotor(0, 150);
  delay(1000);
  while (true) {}

  unsigned long now = millis();
  unsigned long dt_ms = now - lastTime;
  if (dt_ms < CONTROL_INTERVAL_MS) return;
  float dt = dt_ms / 1000.0;
  lastTime = now;

  float error = readPositionError();
  // distance = readUltrasonicDistance();

  if (isPaused) {
    return;
  }

  if (!isClimbing && distance <= 4 && now >= 1000) {
    isClimbing = true;
    stairClimbAttempts = 0;
    Serial.println("Stair detected! Starting climb sequence...");
  }

  if (isClimbing && stairClimbAttempts < 3) {
    
    stairClimbAttempts++;

    Serial.print("Climbing attempt #");
    Serial.println(stairClimbAttempts);

    approachStair();

    setMotor(0, 0);
    delay(PAUSE_BEFORE);

    String lastSensorArray = sensorArray;
    for (int pwm = 0; pwm <= 255; pwm += 5) {
      setMotor(pwm, pwm);
      float currentTime = millis();
      while (millis() - currentTime < 2000 / (255 / 5)) {
        readSensors();
        if (!isPaused && !waitingConfirmation && sensorArray == "11111") {
          detectTime = currentTime;
          waitingConfirmation = true;
        }

        if (waitingConfirmation && currentTime - detectTime >= 50) {
          readSensors();
          if (sensorArray == "00000") {
            setMotor(0, 0);
            isPaused = true;
            return;
          }
          waitingConfirmation = false;
        }
      }
    }

    delay(300);
    return;
  } else if (isClimbing && stairClimbAttempts >= 3) {
    isClimbing = false;
    Serial.println("Finished stair climb sequence.");
  }


  float correction = computePID(error, dt);
  applyMotorCorrection(correction);
}

void readSensors() {
  sensorArray = "";
  for (int i = 0; i < 5; i++) {
    int value = analogRead(SENSORS_PIN[i]);
    sensors[i] = value;
    if (sensorArray == "0011" && i == 4) {
      value += 50;
    }
    char cval = value > (sensor_max[i] - sensor_min[i]) / 5 * 1 + sensor_min[i] ? '1' : '0';
    sensorArray += cval;
  }
}

float readPositionError() {
  readSensors(); 
  int sum = 0;
  int count = 0;
  for (int i = 0; i < 5; i++) {
    if (sensorArray.charAt(i) == '1') {
      sum += weights[i];
      count++;
    }
  }
  if (count > 0) {
    float err = (float)sum / count;
    lastErrorDirection = (err > 0) ? 1 : (err < 0 ? -1 : 0);
    return err;
  } else {
    return lastErrorDirection * 2.5;
  }
}

float readUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  long distance = duration * 0.034 / 2;
  return distance;
}

void approachStair() {
  setMotor(APPROACH_SPEED, APPROACH_SPEED);
  delay(APPROACH_TIME);
}

float computePID(float error, float dt) {
  // Proportional term
  float P = Kp * error;

  // Integral term with anti-windup
  integralTerm += Ki * error * dt;
  integralTerm = constrain(integralTerm, -INTEGRAL_MAX, INTEGRAL_MAX);
  float I = integralTerm;

  // Derivative term (filtered)
  float rawD = (error - prevError) / dt;
  float alpha = dt / (D_FILTER_TAU + dt);
  float derivative = prevDerivative + alpha * (rawD - prevDerivative);
  float D = Kd * derivative;

  // Update history
  prevError = error;
  prevDerivative = derivative;

  return P + I + D;
}

void applyMotorCorrection(float correction) {
  int leftSpeed = constrain(BASE_SPEED + correction, REVERSE_MAX_SPEED, MAX_SPEED);
  int rightSpeed = constrain(BASE_SPEED - correction, REVERSE_MAX_SPEED, MAX_SPEED);

  if (sensorArray == "00100") {
    leftSpeed = STRAIGHT_SPEED;
    rightSpeed = STRAIGHT_SPEED;
  }

  setMotor(leftSpeed, rightSpeed);

  // Optional: debug print
  debugInfo(prevError, Kp * prevError, integralTerm, Kd * prevDerivative, leftSpeed, rightSpeed);
}

void setMotor(int leftSpeed, int rightSpeed) {
  // Động cơ trái (OUT3/OUT4)
  if (leftSpeed >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    leftSpeed = -leftSpeed;
  }
  analogWrite(ENB, constrain(leftSpeed, 0, 255));

  // Động cơ phải (OUT1/OUT2)
  if (rightSpeed >= 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    rightSpeed = -rightSpeed;
  }
  analogWrite(ENA, constrain(rightSpeed, 0, 255));
}


bool isValidLine(String value) {
  return value != "00000" && value != "11111";
}

void debugInfo(float error, float P, float I, float D, int leftSpeed, int rightSpeed) {
  Serial.print("Sensors=");
  Serial.print(sensorArray);
  Serial.print(" Error=");
  Serial.print(error);
  Serial.print(" P=");
  Serial.print(P);
  Serial.print(" I=");
  Serial.print(I);
  Serial.print(" D=");
  Serial.print(D);
  Serial.print(" L=");
  Serial.print(leftSpeed);
  Serial.print(" R=");
  Serial.println(rightSpeed);
}
