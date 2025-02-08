#include "motorcontroller.hpp"  // **Fixed incorrect inclusion syntax**

int count_pulses_left = 0, count_pulses_right = 0;




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

void p2p_pid(MotorController& m1, MotorController& m2, int dist, double error_thresh = 0.3, int ramp_duration = 5000) 
{  // **Fixed incorrect function signature (pass by reference)**
    double setpnt_counts = m1.req_counts(dist);
    Serial.print("REquired counted : ");
    Serial.print(setpnt_counts);
    count_pulses_left = 0;
    count_pulses_right = 0;

    double kp_error = 0.4, kd_error = 1;
    double kp_sync = 0.0, kd_sync = 0;

    double derivative_error = 0, derivative_sync = 0;
    double previous_error = 0, previous_sync = 0;

    unsigned long start_time = millis();
    int initial_speed = 20;

    while (true) {
        unsigned long elapsed_time = millis() - start_time;
        int target_speed = map(constrain(elapsed_time, 0, ramp_duration), 0, ramp_duration, initial_speed, 200);

        double error_distance = abs(setpnt_counts) - abs(count_pulses_left);
        if (setpnt_counts < 0) {
            error_distance = -error_distance;
        }

        int error_sync = 0;

        Serial.print("Distance Error: ");
        Serial.print(error_distance);
        Serial.print("\tSync Error: ");
        Serial.println(error_sync);

        double pv_error = kp_error * error_distance + kd_error * (error_distance - previous_error);
        double pv_sync = kp_sync * error_sync + kd_sync * derivative_sync;

        derivative_error = error_distance - previous_error;
        derivative_sync = error_sync - previous_sync;

        previous_error = error_distance;

        if (pv_error <= error_thresh) {
            m1.rotate(0, 0);  // **Fixed missing speed parameter**
            m2.rotate(0, 0);  // **Fixed missing speed parameter**
            break;
        }
        else if (error_distance > setpnt_counts / 4) {
            Serial.println("Exec");
            m2.rotate(1, min(max((int)(target_speed + pv_sync), initial_speed), 200));  // **Fixed incorrect min/max usage**
            m1.rotate(1, min(max((int)(target_speed - pv_sync), initial_speed), 200));
        }
        else {
            m2.rotate(1, min(max((int)target_speed, initial_speed), 200));
            m1.rotate(1, min(max((int)target_speed, initial_speed), 200));
        }
    }
}

// Global pointers to our MotorController objects (needed for ISRs)
// MotorController* motor1Ptr = nullptr;
// MotorController* motor2Ptr = nullptr;

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

// **Define motor control pins before creating MotorController instances**
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

MotorController motor1(M1_in1, M1_in2, M1_ENC_A, M1_ENC_B, M1_PWM);
MotorController motor2(M2_in1, M2_in2, M2_ENC_A, M2_ENC_B, M2_PWM);

void setup() {
    Serial.begin(9600);
   

    attachInterrupt(digitalPinToInterrupt(M1_ENC_A), encoderInterrupt_left, RISING);
    attachInterrupt(digitalPinToInterrupt(M2_ENC_B), encoderInterrupt_right, RISING);
}

void loop() {
    p2p_pid(motor1, motor2, 25);
    motor1.rotate(0,0);
    motor2.rotate(0,0);
    delay(10000);
}
