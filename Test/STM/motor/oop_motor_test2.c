#include <Arduino.h>

// Constants
#define OPTIMUM_SPEED 150
#define WHEEL_DIAMETER 10.681415

// Pin Definitions (same as your non-OOP code)
#define M1_ENC_A PB3    // Left Encoder A
#define M1_ENC_B PA15   // Left Encoder B
#define M2_ENC_A PB9    // Right Encoder A
#define M2_ENC_B PB8    // Right Encoder B

#define M1_PWM PA0      // Left Motor PWM
#define M1_in1 PA2      // Left Motor Direction 1
#define M1_in2 PA3      // Left Motor Direction 2
#define M2_PWM PA1      // Right Motor PWM
#define M2_in1 PA4      // Right Motor Direction 1
#define M2_in2 PA5      // Right Motor Direction 2

#define PUSH_BUTTON PB15
int count_pulses_left = 0, count_pulses_right =0;

void encoderInterrupt_left() {
    int b = digitalRead(M1_ENC_B);
    if (b > 0) {
        count_pulses_left++;
    }
    else {
        count_pulses_left--;
    }
}

void encoderInterrupt_right() {
    int b = digitalRead(M2_ENC_B);
    if (b > 0) {
        count_pulses_right++;
    }
    else {
        count_pulses_right--;
    }
}


// MotorController Class
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
    

    // Reset the encoder count to zero
    void resetEncoder() {
        countPulses = 0;
    }

    // Return the current encoder count
    long getCount() {
        return countPulses;
    }
};

// Global pointers to our MotorController objects (needed for ISRs)
MotorController* motor1Ptr = nullptr;
MotorController* motor2Ptr = nullptr;

// Global ISR functions call the appropriate object's encoderInterrupt method
// void M1_Encoder_ISR() {
//     if (motor1Ptr) {
//         motor1Ptr->encoderInterrupt();
//     }
// }

// void M2_Encoder_ISR() {
//     if (motor2Ptr) {
//         motor2Ptr->encoderInterrupt();
//     }
// }

// Instantiate MotorController objects using the defined pins
MotorController motor1(M1_in1, M1_in2, M1_ENC_A, M1_ENC_B, M1_PWM);
MotorController motor2(M2_in1, M2_in2, M2_ENC_A, M2_ENC_B, M2_PWM);

void setup() {
    Serial.begin(9600);

    // Set global pointers so the ISRs know which object to call
    motor1Ptr = &motor1;
    motor2Ptr = &motor2;

    // Attach interrupts for each motor's encoder (using the encoder A pins)
    attachInterrupt(digitalPinToInterrupt(M1_ENC_A), encoderInterrupt_left, RISING);
    attachInterrupt(digitalPinToInterrupt(M2_ENC_B), encoderInterrupt_right, RISING);
}

void loop() {
    // Set both motors to rotate forward at full speed (255)
    motor1.rotate(1, 255);
    motor2.rotate(1, 255);

    // Print encoder counts to Serial Monitor every 100ms
    Serial.print("Result A: ");
    Serial.print(count_pulses_left);
    Serial.print("   Results B: ");
    Serial.println(count_pulses_right);

    delay(100);
}
