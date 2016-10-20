// code reads an integer over serial (0-255). then sets PWM
// to recieved value.

int output_pin = 3;
int PWM_val = 0;

void setup() {
  pinMode(output_pin, OUTPUT);
  Serial.begin(9600);

}

void loop() {
  analogWrite(output_pin, PWM_val);
  
  if(Serial.available() > 0)
  {
    PWM_val = Serial.parseInt();
    Serial.print("PWM: ");
    Serial.println(PWM_val, DEC);  
  }
  
}
