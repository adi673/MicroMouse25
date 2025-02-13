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

#define counts_per_rotation 410
#define WHEEL_DIAMETER 4.4

volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

double kp1 = 1.0, kd1 = 0.1; // PID Constants (Adjustable)
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
    spdL = constrain(spdL, -255, 255);
    spdR = constrain(spdR, -255, 255);

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

    while (true) {
        error = setpoint_counts - (*encoderCount);
        Serial.println(*encoderCount);
        Serial.println(error);

        control_signal = kp1 * error + kd1 * (error - previous_error);
        previous_error = error;

        if (abs(error) < 2) { // Threshold to stop
            brake();
            break;
        }

        int speed = constrain(control_signal, -200, 200);
        Motor_SetSpeed(speed, speed);
    }
}

void loop() {
    p2p_pid(25);
    Serial.print("Left Encoder: ");
    Serial.print(leftEncoderCount);
    Serial.print(" | Right Encoder: ");
    Serial.println(rightEncoderCount);
    delay(500);
}
