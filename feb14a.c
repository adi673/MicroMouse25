#include <Arduino.h>
extern "C" void HAL_IncTick(void); 
#define WHEEL_DIAMETER 4.4 // in cm
#define ENCODER_COUNTS_PER_ROTATION 425
#define SETPOINT_DISTANCE 26.0 // Target distance in cm
// Encoder Pins
#define M1_ENC_A PB3  // Encoder A pin for Motor 1 right
#define M1_ENC_B PA15 // Encoder B pin for Motor 1  right
#define M2_ENC_A PB9  // Encoder A pin for Motor 2 left
#define M2_ENC_B PB8  // Encoder B pin for Motor 2 left

// Motor Control Pins
#define M1_PWM PA0   // Motor 1 PWM pin right
#define M1_IN1 PA2   // Motor 1 direction pin 1 right
#define M1_IN2 PA3   // Motor 1 direction pin 2 right 
#define M2_PWM PA1   // Motor 2 PWM pin left
#define M2_IN1 PA4   // Motor 2 direction pin 1 left
#define M2_IN2 PA5   // Motor 2 direction pin 2 left

volatile int count_left = 0, count_right = 0;
double control_left = 0, control_right = 0;
double error_left = 0, error_right = 0;

// Interrupt function for Motor 1 Encoder
void M1_Encoder_Interrupt() {
    int b = digitalRead(M1_ENC_B);
    if (b > 0) {
        count_left++;
    }
    else {
        count_left--;
    }
     Serial.print("Left Encoder: ");
    Serial.println(count_left);
    Serial.print(" control left : ");
        Serial.println(control_left);
                Serial.print("  error left : ");
        Serial.println(error_left);
}

// Interrupt function for Motor 2 Encoder
void M2_Encoder_Interrupt() {
    int b = digitalRead(M2_ENC_A);
    if (b > 0) {
        count_right++;
    }
    else {
        count_right--;
    }
    Serial.print(" | Right Encoder: ");
    Serial.println(count_right);
    Serial.print("  control right : ");
        Serial.println(control_right);
                Serial.print("  errorright : ");
        Serial.print(error_right);
}

// Function to control motor speed and direction
void motorControl(int pwmPin, int in1, int in2, int speed) {
     speed = constrain(speed, -255, 255); // Ensure speed is within valid range

    if (speed > 0) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    } else if (speed < 0) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        speed = -speed;  // Convert speed to positive for PWM
    } else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
    }
    analogWrite(pwmPin, speed);
}

void p2p_pid( double error_thresh = 0.3, int ramp_duration = 5000)
{

 
  motorControl(M1_PWM, M1_IN1, M1_IN2, 100.00);
    motorControl(M2_PWM, M2_IN1, M2_IN2, 100.00);

    while(1){
       if(count_left>400 || count_right>400){
    motorControl(M1_PWM, M1_IN1, M1_IN2, 0.00);
    motorControl(M2_PWM, M2_IN1, M2_IN2, 0.00);
    Serial.println("breakinggggggggggggggggggg");
    delay(2000);
    break;
  }
    }
    // PID Controller. Used to control the speed of the bot. Function

    // double rotations_required = SETPOINT_DISTANCE / (3.1416 * WHEEL_DIAMETER);
    // double setpoint_counts = rotations_required * ENCODER_COUNTS_PER_ROTATION;    


    // double kp_error = 0.4, kd_error = 1;
    // double kp_sync = 0.0, kd_sync = 0;

    // double derivative_error = 0, derivative_sync = 0;
    // double previous_error = 0, previous_sync = 0;

    // unsigned long start_time = millis();
    // double initial_speed = 20; // Initial speed

    // while (true)
    // {
    //     // Calculate elapsed time
    //     unsigned long elapsed_time = millis() - start_time;

    //     // Gradually increase speed over ramp duration
    //     double target_speed = map(constrain(elapsed_time, 0, ramp_duration), 0, ramp_duration, initial_speed, 200);

    //     double error_distance = abs(setpoint_counts) - abs(count_left);
    //     if (setpoint_counts < 0)
    //     {
    //         error_distance = -error_distance;
    //     }

    //     int error_sync = 0;

    //     Serial.print("Distance Error: ");
    //     Serial.print(error_distance);
    //     Serial.print("\tSync Error: ");
    //     Serial.println(error_sync);

    //     double pv_error = kp_error * error_distance + kd_error * (error_distance - previous_error);
    //     double pv_sync = kp_sync * error_sync + kd_sync * derivative_sync;

    //     derivative_error = error_distance - previous_error;
    //     derivative_sync = error_sync - previous_sync;

    //     previous_error = error_distance;

    //     if (pv_error <= error_thresh)
    //     {
    //         motorControl(M1_PWM, M1_IN1, M1_IN2, 0.00);
    //         motorControl(M2_PWM, M2_IN1, M2_IN2, 0.00);
    //         break;
    //     }
    //     else if (error_distance > setpoint_counts / 4)
    //     {
    //         Serial.println("Exec");
    //          motorControl(M1_PWM, M1_IN1, M1_IN2, min(max(target_speed + pv_sync, initial_speed), 200.00));
    //         motorControl(M2_PWM, M2_IN1, M2_IN2, min(max(target_speed - pv_sync, initial_speed), 200.00));
    //     }
    //     else
    //     {
    //          motorControl(M1_PWM, M1_IN1, M1_IN2, min(max(target_speed, initial_speed), 200.00));
    //         motorControl(M2_PWM, M2_IN1, M2_IN2, min(max(target_speed, initial_speed), 200.00));
    //     }
    // }
}
void setup() {
    Serial.begin(9600);
    HAL_InitTick(0);
    // Set encoder pins as input pull-up
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
    attachInterrupt(digitalPinToInterrupt(M2_ENC_B), M2_Encoder_Interrupt, RISING);
}

void loop() {
    // Set motor speed for testing
    // int test_speed = 150;
    
    // motorControl(M1_PWM, M1_IN1, M1_IN2, test_speed);
    // motorControl(M2_PWM, M2_IN1, M2_IN2, test_speed);

    Serial.println("Exec");
    

    // delay(100);  // 100ms delay
    // delay(2000);
    p2p_pid();
    count_left=0;
    count_right=0;
    // p2p_pid();
  // if(count_left >200 || count_right>200){
  //    motorControl(M1_PWM, M1_IN1, M1_IN2, 0);
  //    motorControl(M2_PWM, M2_IN1, M2_IN2, 0);
  //    count_left=0;
  //    count_right=0;
  //    delay(5000);
  // }else{
  //   motorControl(M1_PWM, M1_IN1, M1_IN2, 100);
  //    motorControl(M2_PWM, M2_IN1, M2_IN2, 100);
  
  // // motorControl(M1_PWM, M1_IN1, M1_IN2, 0);
  // //     motorControl(M2_PWM, M2_IN1, M2_IN2, 0);
  //     // delay(2000);
      
  
   
    
}