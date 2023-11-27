#define PIN_BUTTON_WHITE D0
#define PIN_BUTTON_BLUE D1

void setup()
{
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);

  pinMode(PIN_BUTTON_WHITE, INPUT_PULLUP);
  pinMode(PIN_BUTTON_BLUE, INPUT_PULLUP);

  SerialUSB.begin(9600);
}

void loop()
{
  // read the state of the pushbutton value:
  if (digitalRead(PIN_BUTTON_WHITE) == LOW)
  {
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE, LOW);
    SerialUSB.println("Button white pressed");
  }
  else if (digitalRead(PIN_BUTTON_BLUE) == LOW)
  {
    digitalWrite(LED_BLUE, LOW);
    SerialUSB.println("Button blue pressed");
  }
  else
  {

    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE, HIGH);
  }

  delay(100);
}
