#ifndef MOTORCONTROLLER_HPP
#define MOTORCONTROLLER_HPP

#include <Arduino.h>

#define WHEEL_DIAMETER 4.4
#define ENCODER_COUNTS_PER_REVOLUTION 426  // Define your encoder resolution

class MotorController {
public:
    // Motor control and encoder pins
    uint8_t in1, in2, encA, encB, pwm;
    volatile long countPulses;  // Use volatile because it's updated in an ISR

    // Constructor
    MotorController(uint8_t in1, uint8_t in2, uint8_t encA, uint8_t encB, uint8_t pwm);

    // Rotate the motor: dir = 1 for forward, -1 for reverse, 0 for stop
    void rotate(int dir, int speed);

    // This method is called by the ISR to update the encoder count
    void encoderInterrupt();

    // Reset the encoder count to zero
    void resetEncoder();

    // Return the current encoder count
    long getCount();

    // Calculate required encoder counts to travel a certain distance
    double req_counts(int distance);
};

#endif // MOTORCONTROLLER_HPP
