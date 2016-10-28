// code reads an integer over serial (0-255). then sets PWM
// to recieved value.

#define PIN_A 3
#define PIN_B 9
#define PIN_C 10
#define PIN_D 11

int PWM_A = 0;
int PWM_B = 0;
int PWM_C = 0;
int PWM_D = 0;

void setup() {
  pinMode(PIN_A, OUTPUT);
  pinMode(PIN_B, OUTPUT);
  pinMode(PIN_C, OUTPUT);
  pinMode(PIN_D, OUTPUT);
  Serial.begin(115200);
  Serial.setTimeout(1);
}

void loop() {
  analogWrite(PIN_A, PWM_A);
  analogWrite(PIN_B, PWM_B);
  analogWrite(PIN_C, PWM_C);
  analogWrite(PIN_D, PWM_D);
  
  while (Serial.available())
  { 
    if (Serial.find("A"))
    {
      int val = Serial.parseInt();
      PWM_A = constrain(val, 0, 255);
    }
    else break;
    if (Serial.find("B"))
    {
      int val = Serial.parseInt();
      PWM_B = constrain(val, 0, 255);
    }
    else break;
    if (Serial.find("C"))
    {
      int val = Serial.parseInt();
      PWM_C = constrain(val, 0, 255);
    }
    else break;
    if (Serial.find("D"))
    {
      int val = Serial.parseInt();
      PWM_D = constrain(val, 0, 255);
    }
    else break;
  }
}
