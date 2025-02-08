#include "./motorcontroller.hpp"
#include <Arduino.h>

class MotorController {
public:
    // Motor control and encoder pins
    uint8_t in1, in2, encA, encB, pwm;
    volatile long countPulses;  // Use volatile because it's updated in an ISR

    // Constructor
    MotorController(uint8_t in1, uint8_t in2, uint8_t encA, uint8_t encB, uint8_t pwm)
        : in1(in1), in2(in2), encA(encA), encB(encB), pwm(pwm), countPulses(0)
    {
        pinMode(in1, OUTPUT);
        pinMode(in2, OUTPUT);
        pinMode(encA, INPUT);
        pinMode(encB, INPUT);
        pinMode(pwm, OUTPUT);
    }

    // Rotate the motor: dir = 1 for forward, -1 for reverse, 0 for stop
    void rotate(int dir, int speed) {
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

    // This method is called by the ISR to update the encoder count
    void encoderInterrupt() {
        int b = digitalRead(encB);
        if (b > 0) {
            countPulses++;
        }
        else {
            countPulses--;
        }
    }

    // Reset the encoder count to zero
    void resetEncoder() {
        countPulses = 0;
    }

    // Return the current encoder count
    long getCount() {
        return countPulses;
    }
};

double MotorController::req_counts(int distance){
    // Returns the number of encoder counts required to travel a distance
    double rotatn_req = distance / (3.14*WHEEL_DIAMETER);
    double setpnt_counts = rotatn_req * ENCODER_COUNTS_PER_REVOLUTION;
    return setpnt_counts;
}