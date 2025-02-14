#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <Arduino.h>

#define WHEEL_DIAMETER 4.4 // in cm
#define ENCODER_COUNTS_PER_ROTATION 425
#define SETPOINT_DISTANCE 26.0 // Target distance in cm

#define M1_ENC_A 2 // Encoder A pin for Motor 1
#define M1_ENC_B 3 // Encoder B pin for Motor 1
#define M2_ENC_A 4 // Encoder A pin for Motor 2
#define M2_ENC_B 5 // Encoder B pin for Motor 2

#define M1_PWM 6  // Motor 1 PWM pin
#define M1_IN1 7  // Motor 1 direction pin 1
#define M1_IN2 8  // Motor 1 direction pin 2
#define M2_PWM 9  // Motor 2 PWM pin
#define M2_IN1 10 // Motor 2 direction pin 1
#define M2_IN2 11 // Motor 2 direction pin 2

volatile int count_left = 0, count_right = 0;

// Interrupt function for Motor 1 Encoder
void IRAM_ATTR M1_Encoder_Interrupt() {
    if (digitalRead(M1_ENC_B) == HIGH) count_left++;  // Forward
    else count_left--;  // Reverse
}

// Interrupt function for Motor 2 Encoder
void IRAM_ATTR M2_Encoder_Interrupt() {
    if (digitalRead(M2_ENC_B) == HIGH) count_right++;  // Forward
    else count_right--;  // Reverse
}


void motorControl(int pwmPin, int in1, int in2, int speed)
{
    speed = constrain(speed, -255, 255);
    if (speed > 0)
    {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    }
    else if (speed < 0)
    {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        speed = -speed;
    }
    else
    {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW); // Stop the motor
    }
    analogWrite(pwmPin, speed);
}

void brake()
{
    digitalWrite(M1_IN1, LOW);
    digitalWrite(M1_IN2, LOW);
    digitalWrite(M2_IN1, LOW);
    digitalWrite(M2_IN2, LOW);
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

        last_error_left = error_left;
        last_error_right = error_right;

        if (abs(error_left) < 2 && abs(error_right) < 2)
        {
            brake();
            break;
        }

        int speed_left = (int)(control_left > 0 ? control_left : -control_left);
        int speed_right = (int)(control_right > 0 ? control_right : -control_right);

        speed_left = speed_left < 50 ? 50 : (speed_left > 200 ? 200 : speed_left);
        speed_right = speed_right < 50 ? 50 : (speed_right > 200 ? 200 : speed_right);

        motorControl(M1_PWM, M1_IN1, M1_IN2, control_left);
        motorControl(M2_PWM, M2_IN1, M2_IN2, control_right);

        usleep(5000); // Small delay to stabilize PID loop
    }
}
void setup() {
    Serial.begin(9600);
    
    // Set encoder pins as input
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
    attachInterrupt(digitalPinToInterrupt(M2_ENC_A), M2_Encoder_Interrupt, RISING);
}

int main()
{

    pid_control();

    return 0;
}
