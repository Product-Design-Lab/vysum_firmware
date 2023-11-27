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
  while (SerialUSB.available())
  {
    char inChar = (char)SerialUSB.read();
    inputString += inChar;
    if (inChar == '\n')
    {
      SerialUSB.print(inputString);
      inputString = "";
    }
  }
}
