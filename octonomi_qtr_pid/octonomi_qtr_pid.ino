#include <Servo.h>

Servo myservo;

void setup() {
  myservo.attach(5);
  Serial.begin(9600);
}

int x = 0;

void loop() {
  if (Serial.available()) {
    x = Serial.parseInt();
    Serial.println("read");
  }
  myservo.write(x);
}
