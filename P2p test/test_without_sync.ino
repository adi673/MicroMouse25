#include <Arduino.h>
#define WHEEL_DIAMETER 4.4 // in cm
#define ENCODER_COUNTS_PER_ROTATION 425
#define SETPOINT_DISTANCE 26.0 // Target distance in cm
// Encoder Pins
#define M1_ENC_A PB3  // Encoder A pin for Motor 1
#define M1_ENC_B PA15 // Encoder B pin for Motor 1
#define M2_ENC_A PB9  // Encoder A pin for Motor 2
#define M2_ENC_B PB8  // Encoder B pin for Motor 2

// Motor Control Pins
#define M1_PWM PA0   // Motor 1 PWM pin
#define M1_IN1 PA2   // Motor 1 direction pin 1
#define M1_IN2 PA3   // Motor 1 direction pin 2
#define M2_PWM PA1   // Motor 2 PWM pin
#define M2_IN1 PA4   // Motor 2 direction pin 1
#define M2_IN2 PA5   // Motor 2 direction pin 2

volatile int count_left = 0, count_right = 0;

// Interrupt function for Motor 1 Encoder
void M1_Encoder_Interrupt() {
    if (digitalRead(M1_ENC_B) == HIGH) count_left++;  // Forward
    else count_left--;  // Reverse
}

// Interrupt function for Motor 2 Encoder
void M2_Encoder_Interrupt() {
    if (digitalRead(M2_ENC_A) == HIGH) count_right++;  // Forward
    else count_right--;  // Reverse
}

// Function to control motor speed and direction
void motorControl(int pwmPin, int in1, int in2, int speed) {
    speed = constrain(speed, -255, 255); // Ensure speed is within valid range

    if (speed > 0) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    } else if (speed < 0) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        speed = -speed;  // Convert speed to positive for PWM
    } else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
    }
    analogWrite(pwmPin, speed);
}

void pid_control()
{
    double rotations_required = SETPOINT_DISTANCE / (3.1416 * WHEEL_DIAMETER);
    double setpoint_counts = rotations_required * ENCODER_COUNTS_PER_ROTATION;

    double kp = 0.2, kd = 1.0;
    double error_left = 0, error_right = 0;
    double last_error_left = 0, last_error_right = 0;
    double control_left = 0, control_right = 0;

    count_left = 0;
    count_right = 0;

    while (1)
    {
        error_left = setpoint_counts - count_left;
        error_right = setpoint_counts - count_right;

        control_left = kp * error_left + kd * (error_left - last_error_left);
        control_right = kp * error_right + kd * (error_right - last_error_right);

        Serial.print("error left : ");
        Serial.print(error_left);
        Serial.print("errorleft : ");
        Serial.println(control_right);

        Serial.println("");
        Serial.println("");

        
        Serial.print("control left : ");
        Serial.print(control_left);
        Serial.print("control left : ");
        Serial.println(control_right);

        last_error_left = error_left;
        last_error_right = error_right;

        if (abs(error_left) < 2 && abs(error_right) < 2)
        {
            motorControl(M1_PWM, M1_IN1, M1_IN2, 0);
            motorControl(M2_PWM, M2_IN1, M2_IN2, 0);
            break;
        }

        int speed_left = (int)(control_left > 0 ? control_left : -control_left);
        int speed_right = (int)(control_right > 0 ? control_right : -control_right);

        speed_left = speed_left < 50 ? 50 : (speed_left > 200 ? 200 : speed_left);
        speed_right = speed_right < 50 ? 50 : (speed_right > 200 ? 200 : speed_right);

        motorControl(M1_PWM, M1_IN1, M1_IN2, control_left);
        motorControl(M2_PWM, M2_IN1, M2_IN2, control_right);

    }
}

void setup() {
    Serial.begin(115200);
    delay(3000);
    // Set encoder pins as input pull-up
    pinMode(M1_ENC_A, INPUT_PULLUP);
    pinMode(M1_ENC_B, INPUT_PULLUP);
    pinMode(M2_ENC_A, INPUT_PULLUP);
    pinMode(M2_ENC_B, INPUT_PULLUP);
    
    // Set motor control pins as output
    pinMode(M1_PWM, OUTPUT);
    pinMode(M1_IN1, OUTPUT);
    pinMode(M1_IN2, OUTPUT);
    pinMode(M2_PWM, OUTPUT);
    pinMode(M2_IN1, OUTPUT);
    pinMode(M2_IN2, OUTPUT);

    // Attach interrupts for encoders
    attachInterrupt(digitalPinToInterrupt(M1_ENC_A), M1_Encoder_Interrupt, RISING);
    attachInterrupt(digitalPinToInterrupt(M2_ENC_B), M2_Encoder_Interrupt, RISING);
}

void loop() {
    // Set motor speed for testing
    // int test_speed = 150;
    
    // motorControl(M1_PWM, M1_IN1, M1_IN2, test_speed);
    // motorControl(M2_PWM, M2_IN1, M2_IN2, test_speed);

    // Serial.print("Left Encoder: ");
    // Serial.print(count_left);
    // Serial.print(" | Right Encoder: ");
    // Serial.println(count_right);

    // delay(100);  // 100ms delay

    pid_control();
}