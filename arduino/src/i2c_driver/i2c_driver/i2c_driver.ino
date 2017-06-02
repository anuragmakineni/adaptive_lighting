#include <Wire.h>

int slave_addr = 12; //0001100
float reference_voltage = 3.3; //DAC reference voltage



void setup() {
  Serial.begin(115200);
  Serial.setTimeout(10);
  Wire.begin();
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
  // parse into two bytes
  float desired_voltage = com.toFloat();
  float percentage = desired_voltage / reference_voltage;
  int ctl = (int) (percentage * 4095);
  uint8_t upper_byte = ctl >> 8;
  uint8_t lower_byte = (ctl & 0xFF);
  
  // send i2c 
  Wire.beginTransmission(slave_addr);
  Wire.write(upper_byte); //15
  Wire.write(lower_byte); //255
  Wire.endTransmission();
  
  //this now reads the DAC register
  Wire.requestFrom(slave_addr, 2);
  while(Wire.available() == 0);
  int first_byte= Wire.read();
  int second_byte = Wire.read();
  Serial.print(first_byte, BIN);
  Serial.print("\n");
  Serial.print(second_byte, BIN);
  Serial.print("\n\n");

}
