#include "./motorcontroller.hpp"
#include <Arduino.h>

MotorController::MotorController(int in1, int in2, int enca, int encb, int pwm)
    :in1(in1), in2(in2), enca(enca), pwm(pwm), count_pulses(0){
        
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(enca, INPUT);
    pinMode(encb, INPUT);
    pinMode(pwm, OUTPUT);
}

void MotorController::rotate(int speed){
    // Serial.println(speed);
    if (speed < 0){
//        Serial.println(speed);
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        analogWrite(pwm, speed);
    }
    else if (speed > 0){
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        analogWrite(pwm, speed);
    }
    else{
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        analogWrite(pwm, 0);
    }
}

double MotorController::req_counts(int distance){
    // Returns the number of encoder counts required to travel a distance
    double rotatn_req = distance / (3.14*WHEEL_DIAMETER);
    double setpnt_counts = rotatn_req * ENCODER_COUNTS_PER_REVOLUTION;
    return setpnt_counts;
}