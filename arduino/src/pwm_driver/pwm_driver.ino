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
  Serial.setTimeout(10);
}

String command;
void loop() {
  if (Serial.available())
  {
    char c = Serial.read();
    
    if (c == '\n')
    {
      parseCommand(command);
      command = "";
    }
    else
    {
      command += c;
    }
  }
}

void parseCommand(String com)
{
  String str1 = com.substring(1, 4);
  String str2 = com.substring(5, 8);
  String str3 = com.substring(9, 12);
  String str4 = com.substring(13);
  
  PWM_A = str1.toInt();
  PWM_B = str2.toInt();
  PWM_C = str3.toInt();
  PWM_D = str4.toInt();
  
  analogWrite(PIN_A, PWM_A);
  analogWrite(PIN_B, PWM_B);
  analogWrite(PIN_C, PWM_C);
  analogWrite(PIN_D, PWM_D);
  
  Serial.print(PWM_D);
}
