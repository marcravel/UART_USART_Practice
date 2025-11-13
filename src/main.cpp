#include <Arduino.h>

bool ledState = 1;
// put function declarations here:
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {

  digitalWrite(LED_BUILTIN, ledState);   // turn the LED on (HIGH is the voltage level)
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {   // Check if data is received
    uint8_t data = Serial.read();       // Read one byte
    Serial.print("Received: ");
    Serial.println(data, DEC); // Print the received byte as a decimal number
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);    // turn the LED off by making the voltage LOW

  }
}

// put function definitions here:
