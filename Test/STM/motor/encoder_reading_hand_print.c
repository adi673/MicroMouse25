#include <Arduino.h>

// Encoder Pins
#define M1_ENC_A PB3    // Left Encoder A
#define M1_ENC_B PA15   // Left Encoder B
#define M2_ENC_A PB9    // Right Encoder A
#define M2_ENC_B PB8    // Right Encoder B

// Motor Control Pins
#define M1_PWM PA0    // Left Motor PWM
#define M2_PWM PA1    // Right Motor PWM
#define M1_in1 PA2     // Left Motor Direction 1
#define M1_in2 PA3     // Left Motor Direction 2
#define M2_in1 PA4     // Right Motor Direction 1
#define M2_in2 PA5     // Right Motor Direction 2

// Variables to store encoder counts
volatile int leftEncoderCount = 0;
volatile int rightEncoderCount = 0;

void leftEncoderISR() {
    if (digitalRead(M1_ENC_A) == digitalRead(M1_ENC_B)) {
        leftEncoderCount++;
    } else {
        leftEncoderCount--;
    }
}

void rightEncoderISR() {
    if (digitalRead(M2_ENC_A) == digitalRead(M2_ENC_B)) {
        rightEncoderCount++;
    } else {
        rightEncoderCount--;
    }
}

void Motor_SetSpeed(int spdL, int spdR){
    if (spdL == 0){
        digitalWrite(M1_in1, LOW);
        digitalWrite(M1_in2, LOW);
        analogWrite(M1_PWM, 0);
    }
    else if (spdL < 0){
        analogWrite(M1_PWM, -spdL);
        digitalWrite(M1_in1, LOW);
        digitalWrite(M1_in2, HIGH);
    }
    else{
        analogWrite(M1_PWM, spdL);
        digitalWrite(M1_in1, HIGH);
        digitalWrite(M1_in2, LOW);
    }

    if (spdR == 0){
        digitalWrite(M2_in1, LOW);
        digitalWrite(M2_in2, LOW);
        analogWrite(M2_PWM, 0);
    }
    else if (spdR < 0){
        analogWrite(M2_PWM, -spdR);
        digitalWrite(M2_in1, LOW);
        digitalWrite(M2_in2, HIGH);
    }
    else{
        analogWrite(M2_PWM, spdR);
        digitalWrite(M2_in1, HIGH);
        digitalWrite(M2_in2, LOW);
    }
}

void brake(){
    digitalWrite(M1_in1, LOW);
    digitalWrite(M1_in2, LOW);
    digitalWrite(M2_in1, LOW);
    digitalWrite(M2_in2, LOW);
}

void setup() {
    delay(5000);  // Wait for 5 seconds before starting
    Serial.begin(115200);
    pinMode(M1_ENC_A, INPUT_PULLUP);
    pinMode(M1_ENC_B, INPUT_PULLUP);
    pinMode(M2_ENC_A, INPUT_PULLUP);
    pinMode(M2_ENC_B, INPUT_PULLUP);
    pinMode(M1_PWM, OUTPUT);
    pinMode(M2_PWM, OUTPUT);
    pinMode(M1_in1, OUTPUT);
    pinMode(M1_in2, OUTPUT);
    pinMode(M2_in1, OUTPUT);
    pinMode(M2_in2, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(M1_ENC_A), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(M2_ENC_B), rightEncoderISR, CHANGE);
}

void loop() {
    Motor_SetSpeed(50, 50);  // Increased speed for better PWM range
    Serial.print("Left Encoder: ");
    Serial.print(leftEncoderCount);
    Serial.print(" | Right Encoder: ");
    Serial.println(rightEncoderCount);
    delay(500);
}
