#include <PID_v1.h>

// Motor Pins
#define LEFT_PWM 5
#define RIGHT_PWM 6
#define LEFT_DIR 7
#define RIGHT_DIR 8

// Encoder Pins
#define LEFT_ENC_A 2
#define LEFT_ENC_B 3
#define RIGHT_ENC_A 4
#define RIGHT_ENC_B 9

// Encoder counts per cm (Adjust based on motor specs)
#define WHEEL_DIAMETER_CM 4.0   // Example: 4 cm wheel diameter
#define COUNTS_PER_REV 360      // Example: 360 encoder counts per revolution

#define COUNTS_PER_CM (COUNTS_PER_REV / (3.14159 * WHEEL_DIAMETER_CM))
#define TARGET_DISTANCE_CM 26
#define TARGET_COUNTS (TARGET_DISTANCE_CM * COUNTS_PER_CM)

// Encoder counts
volatile long leftCount = 0, rightCount = 0;

// PID variables
double leftInput, leftOutput, leftSetpoint;
double rightInput, rightOutput, rightSetpoint;
double syncInput, syncOutput;  // For synchronization correction

// PID tuning parameters (adjust these)
double Kp = 5.0, Ki = 0.8, Kd = 1.2;  // Position control gains
double Kp_sync = 2.0, Ki_sync = 0.2, Kd_sync = 0.1;  // Sync PID gains

// PID Controllers
PID leftPID(&leftInput, &leftOutput, &leftSetpoint, Kp, Ki, Kd, DIRECT);
PID rightPID(&rightInput, &rightOutput, &rightSetpoint, Kp, Ki, Kd, DIRECT);
PID syncPID(&syncInput, &syncOutput, 0.0, Kp_sync, Ki_sync, Kd_sync, DIRECT);


// Updated motor control function
// Motor Control Function (Proper H-Bridge Implementation)
void setMotorSpeed(int pwmPin, int in1, int in2, int speed) {
    speed = constrain(speed, -255, 255);

    if (speed > 0) {
        digitalWrite(in1, HIGH);   // Forward
        digitalWrite(in2, LOW);
        analogWrite(pwmPin, speed);
    } else if (speed < 0) {
        digitalWrite(in1, LOW);    // Reverse
        digitalWrite(in2, HIGH);
        analogWrite(pwmPin, -speed);
    } else {
        digitalWrite(in1, LOW);    // Stop (Brake mode)
        digitalWrite(in2, LOW);
        analogWrite(pwmPin, 0);
    }
}


// Stop motors
void stopMotors() {
    setMotorSpeed(LEFT_PWM, LEFT_DIR, 0);
    setMotorSpeed(RIGHT_PWM, RIGHT_DIR, 0);
}



void leftEncoderISR() {
    if (digitalRead(LEFT_ENC_B) == HIGH) leftCount++;
    else leftCount--;
}

void rightEncoderISR() {
    if (digitalRead(RIGHT_ENC_B) == HIGH) rightCount++;
    else rightCount--;
}

void setup() {
    Serial.begin(115200);

    pinMode(LEFT_PWM, OUTPUT);
    pinMode(RIGHT_PWM, OUTPUT);
    pinMode(LEFT_DIR, OUTPUT);
    pinMode(RIGHT_DIR, OUTPUT);
    
    pinMode(LEFT_ENC_A, INPUT);
    pinMode(LEFT_ENC_B, INPUT);
    pinMode(RIGHT_ENC_A, INPUT);
    pinMode(RIGHT_ENC_B, INPUT);

    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);

    leftSetpoint = TARGET_COUNTS;
    rightSetpoint = TARGET_COUNTS;
    
    leftPID.SetMode(AUTOMATIC);
    rightPID.SetMode(AUTOMATIC);
    syncPID.SetMode(AUTOMATIC);
}

void loop() {
    leftInput = leftCount;
    rightInput = rightCount;

    leftPID.Compute();
    rightPID.Compute();

    // Synchronization PID: Minimize difference between left and right encoder counts
    syncInput = leftCount - rightCount;
    syncPID.Compute();

    int leftMotorSpeed = constrain(leftOutput - syncOutput, -255, 255);
    int rightMotorSpeed = constrain(rightOutput + syncOutput, -255, 255);

    setMotorSpeed(LEFT_PWM, LEFT_DIR, leftMotorSpeed);
    setMotorSpeed(RIGHT_PWM, RIGHT_DIR, rightMotorSpeed);

    // Stop when both are within a small error range
    if (abs(leftSetpoint - leftCount) < 5 && abs(rightSetpoint - rightCount) < 5) {
        stopMotors();
        Serial.println("Target Reached!");
        while (1); // Stop execution
    }
}

