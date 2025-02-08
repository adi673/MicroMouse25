#define OPTIMUM_SPEED 150
#define WHEEL_DIAMETER 10.681415

// Encoder Pins
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

// Push Button
#define PUSH_BUTTON PB15

int m1_count_pulses = 0;
int m2_count_pulses = 0;

void setup()
{
  Serial.begin(9600);                // activates the serial communication
  pinMode(M1_ENC_A, INPUT); // sets the Encoder_output_A pin as the input
  pinMode(M1_ENC_B, INPUT); // sets the Encoder_output_B pin as the input
  pinMode(M1_PWM, OUTPUT);
  pinMode(M1_in1, OUTPUT);
  pinMode(M1_in2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(M1_ENC_A), M1_Encoder_Interrupt, RISING);

  pinMode(M2_ENC_A, INPUT); // sets the Encoder_output_A pin as the input
  pinMode(M2_ENC_B, INPUT); // sets the Encoder_output_B pin as the input
  pinMode(M2_PWM, OUTPUT);
  pinMode(M2_in1, OUTPUT);
  pinMode(M2_in2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(M2_ENC_A), M2_Encoder_Interrupt, RISING);

  
  

  
}

void loop()
{
  digitalWrite(M1_in1, HIGH);
  digitalWrite(M1_in2, LOW);

  digitalWrite(M2_in1, HIGH);
  digitalWrite(M2_in2, LOW);

  analogWrite(M1_PWM, 255);
  analogWrite(M2_PWM, 255);
  
  Serial.println("Result A: ");
  Serial.println(m1_count_pulses);
  Serial.print("Results B:");
  Serial.println(m2_count_pulses);
  delay(100);
}

void M1_Encoder_Interrupt()
{
  int b = digitalRead(M1_ENC_B);
  if (b > 0)
  {
    m1_count_pulses++;
  }
  else
  {
    m1_count_pulses--;
  }
}
void M2_Encoder_Interrupt()
{
  int c = digitalRead(M2_ENC_B);
  if (c > 0)
  {
    m2_count_pulses++;
  }
  else
  {
    m2_count_pulses--;
  }
}