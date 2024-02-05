void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  // high means led off
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);

}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED_RED, LOW);  // turn the LED on (HIGH is the voltage level)
  delay(1000);                      // wait for a second
  digitalWrite(LED_RED, HIGH);   // turn the LED off by making the voltage LOW
  delay(1000);
  
  digitalWrite(LED_GREEN, LOW);  // turn the LED on (HIGH is the voltage level)
  delay(1000);                      // wait for a second
  digitalWrite(LED_GREEN, HIGH);   // turn the LED off by making the voltage LOW
  delay(1000);

  digitalWrite(LED_BLUE, LOW);  // turn the LED on (HIGH is the voltage level)
  delay(1000);                      // wait for a second
  digitalWrite(LED_BLUE, HIGH);   // turn the LED off by making the voltage LOW
  delay(1000);
  
  
}
