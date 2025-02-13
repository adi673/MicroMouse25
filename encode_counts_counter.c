#include <Arduino.h>

// Define Motor & Encoder Pins
#define PWM_PIN     PA0  // PWM output (Use Timer-based PWM)
#define DIR_PIN1    PA2  // Motor direction control 1
#define DIR_PIN2    PA3  // Motor direction control 2
#define ENC_A       PB3  // Encoder Channel A (Interrupt)
#define ENC_B       PA15 // Encoder Channel B

// Motor & Encoder Parameters
#define WHEEL_DIAMETER 0.03  // Wheel diameter in meters
#define CPR 206              // Encoder counts per revolution
#define GEAR_RATIO 298       // Gear ratio

// Velocity Calculation Variables
volatile int encoderCount = 0;
int lastEncoderCount = 0;
unsigned long lastTime = 0;

// PWM Speed (0-255)
#define PWM_VALUE 100

void encoderISR() {
    if (digitalRead(ENC_A) == digitalRead(ENC_B)) {
        encoderCount++;
    } else {
        encoderCount--;
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(ENC_B, INPUT_PULLUP);
    pinMode(PWM_PIN, OUTPUT);
    pinMode(DIR_PIN1, OUTPUT);
    pinMode(DIR_PIN2, OUTPUT);
    
    attachInterrupt(digitalPinToInterrupt(ENC_A), encoderISR, CHANGE);

    // Set Motor Direction (Forward)
    digitalWrite(DIR_PIN1, HIGH);
    digitalWrite(DIR_PIN2, LOW);

    // Apply PWM
    analogWrite(PWM_PIN, PWM_VALUE);

    lastTime = millis();
}

void loop() {
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastTime) / 1000.0; // Convert to seconds

    // // Calculate velocity
    // int countDiff = encoderCount - lastEncoderCount;
    // float wheelCircumference = PI * WHEEL_DIAMETER;
    // float velocity = (countDiff / 16) * wheelCircumference / deltaTime; // m/s

    // Print values
    Serial.print("Encoder Count: ");
    Serial.print(encoderCount);
    // Serial.print(" | Velocity: ");
    // Serial.print(velocity, 4); // Print velocity with 4 decimal places
    // Serial.println(" m/s");

    // Update last values
    lastEncoderCount = encoderCount;
    lastTime = currentTime;

    delay(100); // Update every 100ms
}