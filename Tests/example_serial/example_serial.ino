#include <Adafruit_TinyUSB.h> // for Serial
String inputString = ""; // a String to hold incoming data

void setup()
{
  // initialize serial:
  Serial.begin(9600);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);
}

void loop()
{
  serialEvent();

  delay(100);
}

void serialEvent()
{
  while (Serial.available())
  {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n')
    {
      Serial.print(inputString);
      inputString = "";
    }
  }
}
