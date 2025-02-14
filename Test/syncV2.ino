#include <PID_v1.h>

// Motor Pins
#define LEFT_PWM PA0
#define RIGHT_PWM PA1
#define LEFT_DIR1 PA2
#define LEFT_DIR2 PA3
#define RIGHT_DIR1 PA4
#define RIGHT_DIR2 PA5

// Encoder Pins
#define LEFT_ENC_A PB3
#define LEFT_ENC_B PA15
#define RIGHT_ENC_A PB9
#define RIGHT_ENC_B PB8 

// Encoder counts per cm (Adjust based on motor specs)
#define WHEEL_DIAMETER_CM 4.4   // Example: 4 cm wheel diameter
#define COUNTS_PER_REV 420     // Example: 360 encoder counts per revolution
#define COUNTS_PER_CM (COUNTS_PER_REV / (3.14159 * WHEEL_DIAMETER_CM))

// Encoder counts
volatile long leftCount = 0, rightCount = 0;

// PID variables
double leftInput, leftOutput, leftSetpoint;
double rightInput, rightOutput, rightSetpoint;
double syncInput, syncOutput;  // For synchronization correction

// PID tuning parameters
double Kp = 5.0, Ki = 0.8, Kd = 1.2;  // Position control gains
double Kp_sync = 0.0, Ki_sync = 0.2, Kd_sync = 0;  // Sync PID gains

// PID Controllers
PID leftPID(&leftInput, &leftOutput, &leftSetpoint, Kp, Ki, Kd, DIRECT);
PID rightPID(&rightInput, &rightOutput, &rightSetpoint, Kp, Ki, Kd, DIRECT);
PID syncPID(&syncInput, &syncOutput, 0.0, Kp_sync, Ki_sync, Kd_sync, DIRECT);

// Motor Control Function
void setMotorSpeed(int pwmPin, int in1, int in2, int speed) {
    speed = constrain(speed, -255, 255);
    if (speed > 0) {
        digitalWrite(in1, HIGH);   
        digitalWrite(in2, LOW);    
        analogWrite(pwmPin, speed);
    } else if (speed < 0) {
        digitalWrite(in1, LOW);    
        digitalWrite(in2, HIGH);   
        analogWrite(pwmPin, -speed);
    } else {
        digitalWrite(in1, LOW);    
        digitalWrite(in2, LOW);    
        analogWrite(pwmPin, 0);
    }
}

// Stop motors
void stopMotors() {
    setMotorSpeed(LEFT_PWM, LEFT_DIR1, LEFT_DIR2, 0);
    setMotorSpeed(RIGHT_PWM, RIGHT_DIR1, RIGHT_DIR2, 0);
}

// Encoder ISR
void leftEncoderISR() { leftCount += (digitalRead(LEFT_ENC_B) == HIGH) ? 1 : -1; }
void rightEncoderISR() { rightCount += (digitalRead(RIGHT_ENC_B) == HIGH) ? 1 : -1; }

void setup() {
    Serial.begin(115200);
    pinMode(LEFT_PWM, OUTPUT);
    pinMode(RIGHT_PWM, OUTPUT);
    pinMode(LEFT_DIR1, OUTPUT);
    pinMode(LEFT_DIR2, OUTPUT);
    pinMode(RIGHT_DIR1, OUTPUT);
    pinMode(RIGHT_DIR2, OUTPUT);
    pinMode(LEFT_ENC_A, INPUT);
    pinMode(LEFT_ENC_B, INPUT);
    pinMode(RIGHT_ENC_A, INPUT);
    pinMode(RIGHT_ENC_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);
    leftPID.SetMode(AUTOMATIC);
    rightPID.SetMode(AUTOMATIC);
    syncPID.SetMode(AUTOMATIC);
}

void p2p_pid(double distance_cm) {
    long targetCounts = distance_cm * COUNTS_PER_CM;
    leftSetpoint = targetCounts;
    rightSetpoint = targetCounts;
    leftCount = 0;
    rightCount = 0;
    
    while (true) {
        leftInput = leftCount;
        rightInput = rightCount;
        leftPID.Compute();
        rightPID.Compute();
        syncInput = leftCount - rightCount;
        syncPID.Compute();
        int leftMotorSpeed = constrain(leftOutput - syncOutput, -255, 255);
        int rightMotorSpeed = constrain(rightOutput + syncOutput, -255, 255);
        setMotorSpeed(LEFT_PWM, LEFT_DIR1, LEFT_DIR2, leftMotorSpeed);
        setMotorSpeed(RIGHT_PWM, RIGHT_DIR1, RIGHT_DIR2, rightMotorSpeed);
        if (abs(leftSetpoint - leftCount) < 5 && abs(rightSetpoint - rightCount) < 5) {
            stopMotors();
            Serial.println("Target Reached!");
            break;
        }
    }
}

void loop() {
    p2p_pid(26); // Call with the desired distance in cm
    delay(5000);
}
