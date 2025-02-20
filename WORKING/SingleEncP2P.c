#define M1_ENC_A PB3  // Left Encoder A
#define M1_ENC_B PA15 // Left Encoder B
#define M2_ENC_A PB9  // Right Encoder A
#define M2_ENC_B PB8  // Right Encoder B

// Motor Control Pins
#define M1_PWM PA0 // Left Motor PWM
#define M2_PWM PA1 // Right Motor PWM
#define M1_in1 PA2 // Left Motor Direction 1
#define M1_in2 PA3 // Left Motor Direction 2
#define M2_in1 PA4 // Right Motor Direction 1
#define M2_in2 PA5 // Right Motor Direction 2

#define counts_per_rotation 415
#define WHEEL_DIAMETER 4.4

volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

double kp1 = 0.3, kd1 = 1.0; // PID Constants (Adjustable)
double maxerror = 500;        // Max error for normalization

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

void Motor_SetSpeed(int spdL, int spdR) {
    spdL = constrain(spdL, -200, 170);
    spdR = constrain(spdR, -200, 170);

    if (spdL == 0) {
        digitalWrite(M1_in1, LOW);
        digitalWrite(M1_in2, LOW);
        analogWrite(M1_PWM, 0);
    } else if (spdL < 0) {
        digitalWrite(M1_in1, LOW);
        digitalWrite(M1_in2, HIGH);
        analogWrite(M1_PWM, abs(spdL));
    } else {
        digitalWrite(M1_in1, HIGH);
        digitalWrite(M1_in2, LOW);
        analogWrite(M1_PWM, spdL);
    }

    if (spdR == 0) {
        digitalWrite(M2_in1, LOW);
        digitalWrite(M2_in2, LOW);
        analogWrite(M2_PWM, 0);
    } else if (spdR < 0) {
        digitalWrite(M2_in1, LOW);
        digitalWrite(M2_in2, HIGH);
        analogWrite(M2_PWM, abs(spdR));
    } else {
        digitalWrite(M2_in1, HIGH);
        digitalWrite(M2_in2, LOW);
        analogWrite(M2_PWM, spdR);
    }
}

void brake() {
    digitalWrite(M1_in1, LOW);
    digitalWrite(M1_in2, LOW);
    digitalWrite(M2_in1, LOW);
    digitalWrite(M2_in2, LOW);
}

void setup() {
    delay(5000);
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
    attachInterrupt(digitalPinToInterrupt(M2_ENC_A), rightEncoderISR, CHANGE);
}

void p2p_pid(int dist) {
    double rotations_required = dist / (3.14 * WHEEL_DIAMETER);
    double setpoint_counts = rotations_required * counts_per_rotation;

    volatile long *encoderCount = &leftEncoderCount;
    long previous_error = 0;
    double error, control_signal;

    int min_speed = 50; // Minimum speed required to overcome friction

    while (true) {
        error = setpoint_counts - (*encoderCount);
        Serial.print("Encoder Count = ");
        Serial.println(*encoderCount);
        Serial.print("Error = ");
        Serial.println(error);

        control_signal = kp1 * error + kd1 * (error - previous_error);
        previous_error = error;

        int speed = constrain(control_signal, -200, 200);

        // Ensure minimum speed is applied
        if (speed > 0 && speed < min_speed) speed = min_speed;
        if (speed < 0 && speed > -min_speed) speed = -min_speed;

        Motor_SetSpeed(speed, speed);

        // Stop when error is very small
        if (abs(error) < 2) { 
            brake();
            break;
        }
    }
}



void loop() {
    while (true) {
        leftEncoderCount = 0;  // Reset encoders before each new move
        rightEncoderCount = 0;
        Serial.print("Left encoder count=");
        Serial.println(leftEncoderCount);
        Serial.print("Right encoder count=");
        Serial.println(rightEncoderCount);
        p2p_pid(25);  // Move 25 cm forward
        delay(5000);    // Small delay before next move
    }
}