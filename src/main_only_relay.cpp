/*#include <Arduino.h>

#define RELAY1 12
#define RELAY2 13
#define RELAY3 14
#define RELAY4 15
#define RELAY5 27

void setup() {
  Serial.begin(9600);
  while (! Serial);

  Serial.println();
  Serial.println();

  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);
  pinMode(RELAY3, OUTPUT);
  pinMode(RELAY4, OUTPUT);
  pinMode(RELAY5, OUTPUT);
  digitalWrite(RELAY1, HIGH);
  digitalWrite(RELAY2, HIGH);
  digitalWrite(RELAY3, HIGH);
  digitalWrite(RELAY4, HIGH);
  digitalWrite(RELAY5, HIGH);
}

void loop() {
  Serial.print("Pin activated : ");
  Serial.println(RELAY1);
  digitalWrite(RELAY1, LOW);
  delay(300);
  digitalWrite(RELAY1, HIGH);

  delay(3000);
}
*/