#include "./motorcontroller.hpp"

int count_pulses_left = 0, count_pulses_right =0;

// void encoderInterrupt_left(){
//     int b = digitalRead(5);
//     if (b > 0)
//     {
//         count_pulses_left++;
//     }
//     else
//     {
//         count_pulses_left--;
//     }
// }

// void encoderInterrupt_right(){
//     int b = digitalRead(4);
//     if (b > 0)
//     {
//         count_pulses_right++;
//     }
//     else
//     {
//         count_pulses_right--;
//     }
// }

void p2p_pid(MotorController m1, MotorController m2, int dist, double error_thresh = 0.3, int ramp_duration = 5000)
{
    // PID Controller. Used to control the speed of the bot. Function

    double setpnt_counts = m1.req_counts(dist); // number of counts required to reach the set point

    count_pulses_left = 0;
    count_pulses_right = 0;

    double kp_error = 0.4, kd_error = 1;
    double kp_sync = 0.0, kd_sync = 0;

    double derivative_error = 0, derivative_sync = 0;
    double previous_error = 0, previous_sync = 0;

    unsigned long start_time = millis();
    int initial_speed = 20; // Initial speed

    while (true)
    {
        // Calculate elapsed time
        unsigned long elapsed_time = millis() - start_time;

        // Gradually increase speed over ramp duration
        int target_speed = map(constrain(elapsed_time, 0, ramp_duration), 0, ramp_duration, initial_speed, 200);

        double error_distance = abs(setpnt_counts) - abs(count_pulses_left);
        if (setpnt_counts < 0)
        {
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

        if (pv_error <= error_thresh)
        {
            m1.rotate(0);
            m2.rotate(0);
            break;
        }
        else if (error_distance > setpnt_counts / 4)
        {
            Serial.println("Exec");
            m2.rotate(min(max(target_speed + pv_sync, initial_speed), 200));
            m1.rotate(min(max(target_speed - pv_sync, initial_speed), 200));
        }
        else
        {
            m2.rotate(min(max(target_speed, initial_speed), 200));
            m1.rotate(min(max(target_speed, initial_speed), 200));
        }
    }
}



MotorController motor1(M1_in1, M1_in2, M1_ENC_A, M1_ENC_B, M1_PWM);
MotorController motor2(M2_in1, M2_in2, M2_ENC_A, M2_ENC_B, M2_PWM);

void setup(){
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(3), encoderInterrupt_left, RISING);
  attachInterrupt(digitalPinToInterrupt(2), encoderInterrupt_right, RISING);
}

void loop()
{
  // Serial.print(count_pulses_left);
  // Serial.print("   ");
  // Serial.println(count_pulses_right);
  p2p_pid(motor1, motor2, 25);
  delay(2000);
}