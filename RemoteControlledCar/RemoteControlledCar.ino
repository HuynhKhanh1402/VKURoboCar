/*******************************************************************************
 * Robot Arm Control with PS2 Controller
 *
 * This program controls a robot with:
 * - 4-wheel base movement using L298N motor driver
 * - 4-DOF robotic arm using servos:
 *   - Base rotation (servo1)
 *   - Forward/backward (servo2)
 *   - Up/down (servo3)
 *   - Gripper (servo4)
 *
 * Controls:
 * - D-pad: Robot movement (forward/back/left/right)
 * - Square/Circle: Rotate base left/right
 * - Triangle/Cross: Move arm forward/backward
 * - L1/R1: Lower/raise arm
 * - L2/R2: Grip/release
 * - Right analog stick: Adjust motor speed
 * - START: Reset arm position
 *******************************************************************************/

#include <PS2X_lib.h>
#include <Servo.h>

/************************** PIN DEFINITIONS **************************/
// PS2 Controller Pins
#define PS2_DAT 5 // Data pin
#define PS2_CMD 4 // Command pin
#define PS2_SEL 3 // Select pin
#define PS2_CLK 2 // Clock pin

// L298N Motor Driver Pins
#define ENA 6  // Enable motor A
#define IN1 7  // Motor A input 1
#define IN2 8  // Motor A input 2
#define IN3 9  // Motor B input 1
#define IN4 10 // Motor B input 2
#define ENB 11 // Enable motor B

/************************** CONSTANTS *******************************/
// Motor Constants
#define MOTOR_SPEED_INIT 205 // Initial motor speed (0-255)
#define MOTOR_SPEED_STEP 10  // Speed adjustment step
#define SERVO_DELAY 10       // Delay between servo movements (ms)

// PS2 Controller Configuration
#define PRESSURES false // Analog button pressures
#define RUMBLE false    // Controller rumble

// Servo Default Positions
#define BASE_CENTER 90    // Center position for base rotation (0-180)
#define ARM_BACK 180      // Default backward position (60-180)
#define ARM_DOWN 0        // Lowest arm position (0-90)
#define GRIPPER_CLOSED 70 // Closed gripper position (80-180)
#define GRIPPER_OPEN 180  // Open gripper position

// Servo Movement Limits
#define BASE_STEP 15     // Degrees to move for base rotation
#define ARM_STEP 15      // Degrees to move for arm forward/back
#define ARM_MIN_ANGLE 90 // Minimum angle for arm forward/back

/************************** GLOBAL VARIABLES ***********************/
PS2X ps2x;     // PS2 controller object
int error = 0; // Controller error status
byte type = 0; // Controller type

// Servo objects and their angles
Servo servo1; // Base rotation servo
Servo servo2; // Forward/back servo
Servo servo3; // Up/down servo
Servo servo4; // Gripper servo

// Current servo angles (initialized to default positions)
int angle1 = BASE_CENTER;    // Base rotation (0-180)
int angle2 = ARM_BACK;       // Forward/back (60-180)
int angle3 = ARM_DOWN;       // Up/down (0-90)
int angle4 = GRIPPER_CLOSED; // Gripper (80-180)

int motorSpeed = MOTOR_SPEED_INIT;

/************************** UTILITY FUNCTIONS **********************/
void debug(String msg)
{
  Serial.println("[DEBUG] " + msg);
}

/*
 * Smoothly moves a servo from current position to target position
 * to prevent jerky movements that could damage the servo
 */
void moveServoSmooth(Servo &servo, int target)
{
  int current = servo.read();
  debug("Servo from " + String(current) + " to " + String(target));

  if (current < target)
  {
    for (int i = current; i <= target; i++)
    {
      servo.write(i);
      delay(SERVO_DELAY);
    }
  }
  else
  {
    for (int i = current; i >= target; i--)
    {
      servo.write(i);
      delay(SERVO_DELAY);
    }
  }
}

/************************** SETUP FUNCTION ************************/
void setup()
{
  Serial.begin(57600);
  delay(300); // Allow time for serial to initialize

  // Initialize PS2 controller
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, PRESSURES, RUMBLE);
  if (error == 0)
  {
    Serial.println("Found Controller, configured successful");
  }
  else
  {
    Serial.println("Error connecting to controller");
    return;
  }

  // Identify controller type
  type = ps2x.readType();
  switch (type)
  {
  case 0:
    Serial.println("Unknown Controller type");
    break;
  case 1:
    Serial.println("DualShock Controller Found");
    break;
  }

  // Initialize and position servos
  servo1.attach(17); // Base rotation xanh
  servo2.attach(16); // Forward/back vang
  servo3.attach(15); // Up/down cam
  servo4.attach(14); // Gripper bien

  // Move servos to initial position
  resetArm();

  // Initialize motor control pins
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  debug("System ready.");
}

/************************** MAIN LOOP ****************************/
void loop()
{
  if (error != 0)
    return;

  ps2x.read_gamepad();

  // Robot Base Movement Control (D-pad)
  if (ps2x.Button(PSB_PAD_UP))
  {
    debug("Forward");
    forward();
  }
  else if (ps2x.Button(PSB_PAD_DOWN))
  {
    debug("Back");
    back();
  }
  else if (ps2x.Button(PSB_PAD_LEFT))
  {
    debug("Left");
    left();
  }
  else if (ps2x.Button(PSB_PAD_RIGHT))
  {
    debug("Right");
    right();
  }
  else
  {
    stopMotors();
  }

  if (ps2x.Button(PSB_SQUARE))
  {
    left();
  }
  if (ps2x.Button(PSB_CIRCLE))
  {
    right();
  }
  if (ps2x.Button(PSB_PAD_UP) && ps2x.Button(PSB_SQUARE))
  {
    debug("Straight Left");
    straightLeft();
  }
  if (ps2x.Button(PSB_PAD_UP) && ps2x.Button(PSB_CIRCLE))
  {
    debug("Straight Right");
    straightRight();
  }
  // Arm Forward/Back Control (Triangle/Cross)
  if (ps2x.Button(PSB_TRIANGLE))
  {
    debug("Arm forward");
    moveArmForward();
  }
  if (ps2x.Button(PSB_CROSS))
  {
    debug("Arm backward");
    moveArmBackward();
  }

  // Arm Height Control (L1/R1)
  if (ps2x.Button(PSB_L1))
  {
    debug("Lower arm");
    lowerArm();
  }
  if (ps2x.Button(PSB_R1))
  {
    debug("Raise arm");
    raiseArm();
  }

  // Gripper Control (L2/R2)
  if (ps2x.Button(PSB_L2))
  {
    debug("Grip");
    gripBall();
  }
  if (ps2x.Button(PSB_R2))
  {
    debug("Release");
    releaseBall();
  }

  // Reset Control (START)
  if (ps2x.Button(PSB_START))
  {
    debug("Reset all");
    resetArm();
    stopMotors();
  }

  // Motor Speed Control (Right analog stick)
  int ry = ps2x.Analog(PSS_RY);
  if (ry < 100)
  { // Stick up
    increaseMotorSpeed();
  }
  else if (ry > 200)
  { // Stick down
    decreaseMotorSpeed();
  }

  delay(50); // Small delay to prevent overwhelming the controller
}

/************************** MOVEMENT FUNCTIONS *******************/
void forward()
{
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void back()
{
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void left()
{
  analogWrite(ENA, motorSpeed - 20);
  analogWrite(ENB, motorSpeed - 20);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void right()
{
  analogWrite(ENA, motorSpeed - 20);
  analogWrite(ENB, motorSpeed - 20);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void stopMotors()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

/************************** ARM CONTROL FUNCTIONS ***************/
void resetArm()
{
  angle1 = BASE_CENTER;
  angle2 = ARM_BACK;
  angle3 = ARM_DOWN;
  angle4 = GRIPPER_CLOSED;

  moveServoSmooth(servo1, angle1);
  moveServoSmooth(servo2, angle2);
  moveServoSmooth(servo3, angle3);
  moveServoSmooth(servo4, angle4);
}

void rotateBaseLeft()
{
  angle1 = min(180, angle1 + BASE_STEP);
  moveServoSmooth(servo1, angle1);
}

void rotateBaseRight()
{
  angle1 = max(0, angle1 - BASE_STEP);
  moveServoSmooth(servo1, angle1);
}

void moveArmForward()
{
  angle2 = max(ARM_MIN_ANGLE, angle2 - ARM_STEP);
  moveServoSmooth(servo2, angle2);
}

void moveArmBackward()
{
  angle2 = min(180, angle2 + ARM_STEP);
  moveServoSmooth(servo2, angle2);
}

void lowerArm()
{
  angle3 = max(0, angle3 - 5);
  moveServoSmooth(servo3, angle3);
}

void raiseArm()
{
  angle3 = min(40, angle3 + 5);
  moveServoSmooth(servo3, angle3);
}

void gripBall()
{
  angle4 = 70;
  moveServoSmooth(servo4, angle4);
}

void releaseBall()
{
  angle4 = 180;
  moveServoSmooth(servo4, angle4);
}

void increaseMotorSpeed()
{
  motorSpeed = min(255, motorSpeed + MOTOR_SPEED_STEP);
  debug("Motor speed: " + String(motorSpeed));
}

void decreaseMotorSpeed()
{
  motorSpeed = max(0, motorSpeed - MOTOR_SPEED_STEP);
  debug("Motor speed: " + String(motorSpeed));
}

void straightLeft() {
  analogWrite(ENA, motorSpeed);  // Only move motor A
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);  // Stop motor B
  digitalWrite(IN4, LOW);
}

void straightRight() {
  analogWrite(ENB, motorSpeed);  // Only move motor B
  digitalWrite(IN1, LOW);  // Stop motor A
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}
