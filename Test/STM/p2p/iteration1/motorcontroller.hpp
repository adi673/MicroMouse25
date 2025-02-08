#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#define WHEEL_DIAMETER 3.4
#define ENCODER_COUNTS_PER_REVOLUTION 650

class MotorController {
public:
    MotorController(int in1, int in2, int enca, int encb, int pwm);
    void rotate(int speed);
    double req_counts(int distance);

    int in1, in2, enca, encb, pwm;
    int count_pulses;
};

#endif