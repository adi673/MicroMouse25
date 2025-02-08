#include "motorcontroller.hpp"  // **Fixed incorrect inclusion syntax**
#include <Arduino.h>

// **Fixed redundant class declaration**
MotorController::MotorController(uint8_t in1, uint8_t in2, uint8_t encA, uint8_t encB, uint8_t pwm)
    : in1(in1), in2(in2), encA(encA), encB(encB), pwm(pwm), countPulses(0)
{
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(encA, INPUT);
    pinMode(encB, INPUT);
    pinMode(pwm, OUTPUT);
}

// Rotate the motor: dir = 1 for forward, -1 for reverse, 0 for stop
void MotorController::rotate(int dir, int speed) {  // **Fixed function definition**
    if (dir == 1) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    }
    else if (dir == -1) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    }
    else { // Stop
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
    }
    analogWrite(pwm, speed);
}

// ISR handler for encoders
void MotorController::encoderInterrupt() {
    int b = digitalRead(encB);
    if (b > 0) {
        countPulses++;
    }
    else {
        countPulses--;
    }
}

// Reset the encoder count to zero
void MotorController::resetEncoder() {
    countPulses = 0;
}

// Return the current encoder count
long MotorController::getCount() {
    return countPulses;
}

// **Fixed incorrect function definition**
double MotorController::req_counts(int distance) {
    double rotatn_req = distance / (3.14 * WHEEL_DIAMETER);
    double setpnt_counts = rotatn_req * ENCODER_COUNTS_PER_REVOLUTION;
    Serial.println("Required encoder counts : ");
    Serial.println(setpnt_counts);
    return setpnt_counts;
}
