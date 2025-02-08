// not working

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


class MotorController
{
public:
    uint8_t in1, in2, encA, encB, pwm;
    volatile long countPulses;

    MotorController(uint8_t in1, uint8_t in2, uint8_t encA, uint8_t encB, uint8_t pwm)
        : in1(in1), in2(in2), encA(encA), encB(encB), pwm(pwm), countPulses(0){
        
        pinMode(encA, INPUT);
        pinMode(encB, INPUT);
        pinMode(pwm, OUTPUT);
        pinMode(in1, OUTPUT);
        pinMode(in2, OUTPUT);
        attachInterrupt(digitalPinToInterrupt(encA), encoderInterrupt, RISING);
    }

    void rotate(int dir, int speed)
    {
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

    void encoderInterrupt()
    {
        int b = digitalRead(encB);
        if (b > 0)
        {
            countPulses++;
        }
        else
        {
            countPulses--;
        }
    }

    void resetEncoder()
    {
        countPulses = 0;
    }

    int getCount()
    {
        return countPulses;
    }
};

MotorController motor1(M1_in1, M1_in2, M1_ENC_A, M1_ENC_B, M1_PWM);
MotorController motor2(M2_in1, M2_in2, M2_ENC_A, M2_ENC_B, M2_PWM);

void setup()
{
    Serial.begin(9600);
}

void loop()
{
    motor1.rotate(1, 255);
    motor2.rotate(1, 255);

    motor1.resetEncoder();
    motor2.resetEncoder();

    while (true)
    {
        Serial.print("Result A: ");
        Serial.print(motor1.getCount());
        Serial.print("   Results B:");
        Serial.println(motor2.getCount());
        delay(100);
    }
}