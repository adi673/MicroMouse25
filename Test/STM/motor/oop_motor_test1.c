class MotorController
{
public:
    uint8_t in1, in2, enca, encb, pwm;
    volatile long count_pulses;

    MotorController(uint8_t in1, uint8_t in2, uint8_t encA, uint8_t encB, uint8_t pwm)
        : in1(in1), in2(in2), encA(encA), encB(encB), pwm(pwm), countPulses(0){
        
        pinMode(enca, INPUT);
        pinMode(encb, INPUT);
        pinMode(pwm, OUTPUT);
        pinMode(in1, OUTPUT);
        pinMode(in2, OUTPUT);
        attachInterrupt(digitalPinToInterrupt(enca), encoderInterrupt, RISING);
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
        int b = digitalRead(encb);
        if (b > 0)
        {
            count_pulses++;
        }
        else
        {
            count_pulses--;
        }
    }

    void resetEncoder()
    {
        count_pulses = 0;
    }

    int getCount()
    {
        return count_pulses;
    }
};

MotorController motor1(7, 8, 2, 4, 9);
MotorController motor2(A1, A2, 3, 5, 10);

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