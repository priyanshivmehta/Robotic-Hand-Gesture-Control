#include <Servo.h>

const int numFingers = 5;
const int servoPins[numFingers] = {3, 5, 7, 9, 11};
Servo fingers[numFingers];

const int closedAngle = 120;
const int openAngle = 0;

void setup() {
  Serial.begin(9600);

  for (int i = 0; i < numFingers; i++) {
    fingers[i].attach(servoPins[i]);
    fingers[i].write(closedAngle); // start closed
  }

  Serial.println("Send 5-digit string (0=close, 1=open), e.g., 10110");
}

void loop() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); // remove whitespace/newline

    if (cmd.length() != numFingers) {
      Serial.println("❌ Invalid command length. Must be 5 digits (0 or 1).");
      return;
    }

    bool partialSuccess = false;

    for (int i = 0; i < numFingers; i++) {
      char c = cmd.charAt(i);

      // Reverse logic for finger 5 (index 4 / pin 11)
      bool reversed = (i == 4);

      if (c == '1') {
        fingers[i].write(reversed ? closedAngle : openAngle);
        Serial.print("Finger ");
        Serial.print(i + 1);
        Serial.println(reversed ? ": closed ✅" : ": opened ✅");
        partialSuccess = true;
      } 
      else if (c == '0') {
        fingers[i].write(reversed ? openAngle : closedAngle);
        Serial.print("Finger ");
        Serial.print(i + 1);
        Serial.println(reversed ? ": opened ✅" : ": closed ✅");
        partialSuccess = true;
      } 
      else {
        Serial.print("Finger ");
        Serial.print(i + 1);
        Serial.println(": invalid input ❌");
      }
    }

    if (!partialSuccess) {
      Serial.println("⚠️ No valid commands executed.");
    }
  }
}