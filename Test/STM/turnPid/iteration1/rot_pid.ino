#include "Wire.h"
#include <MPU6050_light.h>
#include <Arduino.h>

#define OPTIMUM_SPEED 150
#define WHEEL_DIAMETER 3.4

class MotorController {
public:
    MotorController(uint8_t in1, uint8_t in2, uint8_t encA, uint8_t encB, uint8_t pwm);
    void rotate(int dir, int speed);
    void brake();

private:
    int in1, in2, pwm;
};

MotorController::MotorController(uint8_t in1, uint8_t in2, uint8_t encA, uint8_t encB, uint8_t pwm) : in1(in1), in2(in2), pwm(pwm) {
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(pwm, OUTPUT);
}

void MotorController::rotate(int dir, int speed) {
     if (dir == 1)
        {
            digitalWrite(in1, HIGH);
            digitalWrite(in2, LOW);
            analogWrite(pwm, speed);
        }
        else if (dir == -1)
        {
            digitalWrite(in1, LOW);
            digitalWrite(in2, HIGH);
            analogWrite(pwm, speed);
        }
        else
        {
            digitalWrite(in1, LOW);
            digitalWrite(in2, LOW);
            analogWrite(pwm, 0);
        }
}

void MotorController::brake() {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
}

class GyroController {
public:
    GyroController();
    void calibrate();
    void update();
    double getAngleZ();
    void gyro_pid(int angle, MotorController &m1, MotorController &m2);

private:
    MPU6050 mpu;
    double kp3 = 0.5, kd3 = 1;
};

GyroController::GyroController() : mpu(Wire) {}

void GyroController::calibrate() {
    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    while (status != 0) {
        Serial.println("Not Connected");
    }
    Serial.println(F("Calculating offsets, do not move MPU6050"));
    mpu.calcOffsets();
    Serial.println("Done!");
}

void GyroController::update() {
    mpu.update();
}

double GyroController::getAngleZ() {
    return mpu.getAngleZ();
}

void GyroController::gyro_pid(int angle, MotorController &m1, MotorController &m2) {
    int curr_angle = int(getAngleZ()) % 360;
    int req_angle = (curr_angle - angle) % 360;

    double error = 0, lasterror = 0, pv = 0;

    while (true) {
        update();
        error = req_angle - (int(getAngleZ()) % 360);
        pv = (kp3 * error) + (kd3 * (error - lasterror));
        lasterror = error;

        if (pv >= -0.1 && pv <= 0.1) {
            m1.brake();
            m2.brake();
            break;
        } else if (pv > 0) {
            int speed = min(max(pv, 50), 200);
            m1.rotate(speed);
            m2.rotate(speed);
        } else {
            int speed = min(max(pv, -200), -50);
            m1.rotate(speed);
            m2.rotate(speed);
        }
    }
}

MotorController motor1(8, 7, 9);
MotorController motor2(A2, A1, 10);
GyroController gyro;

void setup() {
    Serial.begin(9600);
    gyro.calibrate();
}

void loop() {
    gyro.gyro_pid(-90, motor1, motor2);
    Serial.println("Finished");
    motor1.brake();
    motor2.brake();
    delay(2000);
}
