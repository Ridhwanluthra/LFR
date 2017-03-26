#include <NewPing.h>

#define rightMotorF 7
#define rightMotorB 8
#define rightMotorPWM 9
#define leftMotorF 4
#define leftMotorB 5
#define leftMotorPWM 3
#define rightSensor A7
#define forwardSensor A5
#define stby 6
#define led 13

#define s1 A0
#define s2 A1
#define s3 A2
#define s4 A3
#define s5 A4

#define num_sensors 5

#define TRIGGER_PIN  10
#define ECHO_PIN     2
#define MAX_DISTANCE 200

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

void setup() {
  pinMode(rightMotorF, OUTPUT);
  pinMode(rightMotorB, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotorF, OUTPUT);
  pinMode(leftMotorB, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(rightSensor, INPUT);
  pinMode(forwardSensor, INPUT);
  pinMode(led,OUTPUT);
  pinMode(stby,OUTPUT);

  pinMode(s1, INPUT);
  pinMode(s2, INPUT);
  pinMode(s3, INPUT);
  pinMode(s4, INPUT);
  pinMode(s5, INPUT);
  
  // Serial.begin(9600);
}
int ll, l, m , r, rr;
void loop() {
  ll = digitalRead(s1);
  l = digitalRead(s2);
  r = digitalRead(s3);
  m = digitalRead(s4);
  rr = digitalRead(s5);
  

  // if (rr == 0) {
  //   digitalWrite(rightMotorF, HIGH);
  //   digitalWrite(rightMotorB, LOW);
  //   analogWrite(rightMotorPWM, 0);
  //   digitalWrite(leftMotorF, HIGH);
  //   digitalWrite(leftMotorB, LOW);
  //   analogWrite(leftMotorPWM, 0);
  //   digitalWrite(stby,HIGH);
  //   delay(30);

  //   digitalWrite(rightMotorF, HIGH);
  //   digitalWrite(rightMotorB, LOW);
  //   analogWrite(rightMotorPWM, 0);
  //   digitalWrite(leftMotorF, HIGH);
  //   digitalWrite(leftMotorB, LOW);
  //   analogWrite(leftMotorPWM, 100);
  //   digitalWrite(stby,HIGH);
  //   delay(300);

  //   while(m != 0) {
  //     m = digitalRead(s4);
  //     digitalWrite(rightMotorF, HIGH);
  //     digitalWrite(rightMotorB, LOW);
  //     analogWrite(rightMotorPWM, 0);
  //     digitalWrite(leftMotorF, HIGH);
  //     digitalWrite(leftMotorB, LOW);
  //     analogWrite(leftMotorPWM, 100);
  //     digitalWrite(stby,HIGH);
  //   }
  // }

  // if (ll == 0) {
  //   digitalWrite(rightMotorF, HIGH);
  //   digitalWrite(rightMotorB, LOW);
  //   analogWrite(rightMotorPWM, 0);
  //   digitalWrite(leftMotorF, HIGH);
  //   digitalWrite(leftMotorB, LOW);
  //   analogWrite(leftMotorPWM, 0);
  //   digitalWrite(stby,HIGH);
  //   delay(30);

  //   digitalWrite(rightMotorF, HIGH);
  //   digitalWrite(rightMotorB, LOW);
  //   analogWrite(rightMotorPWM, 100);
  //   digitalWrite(leftMotorF, HIGH);
  //   digitalWrite(leftMotorB, LOW);
  //   analogWrite(leftMotorPWM, 0);
  //   digitalWrite(stby,HIGH);
  //   delay(300);

  //   while(m != 0) {
  //     m = digitalRead(s4);
  //     digitalWrite(rightMotorF, HIGH);
  //     digitalWrite(rightMotorB, LOW);
  //     analogWrite(rightMotorPWM, 100);
  //     digitalWrite(leftMotorF, HIGH);
  //     digitalWrite(leftMotorB, LOW);
  //     analogWrite(leftMotorPWM, 0);
  //     digitalWrite(stby,HIGH);
  //   }
  // }




  if (rr == 0) {
    digitalWrite(rightMotorF, HIGH);
    digitalWrite(rightMotorB, LOW);
    analogWrite(rightMotorPWM, 0);
    digitalWrite(leftMotorF, HIGH);
    digitalWrite(leftMotorB, LOW);
    analogWrite(leftMotorPWM, 0);
    digitalWrite(stby,HIGH);
    delay(30);

    digitalWrite(rightMotorF, HIGH);
    digitalWrite(rightMotorB, LOW);
    analogWrite(rightMotorPWM, 0);
    digitalWrite(leftMotorF, HIGH);
    digitalWrite(leftMotorB, LOW);
    analogWrite(leftMotorPWM, 100);
    digitalWrite(stby,HIGH);
    delay(300);

    while(m != 0) {
      m = digitalRead(s4);
      digitalWrite(rightMotorF, HIGH);
      digitalWrite(rightMotorB, LOW);
      analogWrite(rightMotorPWM, 0);
      digitalWrite(leftMotorF, HIGH);
      digitalWrite(leftMotorB, LOW);
      analogWrite(leftMotorPWM, 100);
      digitalWrite(stby,HIGH);
    }
  }

  if (ll == 0) {
    digitalWrite(rightMotorF, HIGH);
    digitalWrite(rightMotorB, LOW);
    analogWrite(rightMotorPWM, 0);
    digitalWrite(leftMotorF, HIGH);
    digitalWrite(leftMotorB, LOW);
    analogWrite(leftMotorPWM, 0);
    digitalWrite(stby,HIGH);
    delay(30);

    m = digitalRead(s4);
    if (m != 0) {
      digitalWrite(rightMotorF, HIGH);
      digitalWrite(rightMotorB, LOW);
      analogWrite(rightMotorPWM, 100);
      digitalWrite(leftMotorF, HIGH);
      digitalWrite(leftMotorB, LOW);
      analogWrite(leftMotorPWM, 0);
      digitalWrite(stby,HIGH);
      delay(300);

      while(m != 0) {
        m = digitalRead(s4);
        digitalWrite(rightMotorF, HIGH);
        digitalWrite(rightMotorB, LOW);
        analogWrite(rightMotorPWM, 100);
        digitalWrite(leftMotorF, HIGH);
        digitalWrite(leftMotorB, LOW);
        analogWrite(leftMotorPWM, 0);
        digitalWrite(stby,HIGH);
      }
    }
  }













  if (l == 0 && m == 0 && r == 0) {
    sstop();
  }
  else if (l == 0 && m == 0 && r == 1) {
    left();
  }
  else if (l == 0 && m == 1 && r == 0) {
    sstop();
  }
  else if (l == 0 && m == 1 && r == 1) {
    soft_left();
  }
  else if (l == 1 && m == 0 && r == 0) {
    right();
  }
  else if (l == 1 && m == 0 && r == 1) {
    forward();
  }
  else if (l == 1 && m == 1 && r == 0) {
    soft_right();
  }
  else if (l == 1 && m == 1 && r == 1) {
    forward();
  }
}

void forward() {
  digitalWrite(rightMotorF, HIGH);
  digitalWrite(rightMotorB, LOW);
  analogWrite(rightMotorPWM, 100);
  digitalWrite(leftMotorF, HIGH);
  digitalWrite(leftMotorB, LOW);
  analogWrite(leftMotorPWM, 100);
  digitalWrite(stby,HIGH);
}

void left() {
  digitalWrite(rightMotorF, HIGH);
  digitalWrite(rightMotorB, LOW);
  analogWrite(rightMotorPWM, 100);
  digitalWrite(leftMotorF, HIGH);
  digitalWrite(leftMotorB, LOW);
  analogWrite(leftMotorPWM, 0);
  digitalWrite(stby,HIGH);
}

void soft_left() {
  digitalWrite(rightMotorF, HIGH);
  digitalWrite(rightMotorB, LOW);
  analogWrite(rightMotorPWM, 100);
  digitalWrite(leftMotorF, HIGH);
  digitalWrite(leftMotorB, LOW);
  analogWrite(leftMotorPWM, 50);
  digitalWrite(stby,HIGH);
}

void right() {
  digitalWrite(rightMotorF, HIGH);
  digitalWrite(rightMotorB, LOW);
  analogWrite(rightMotorPWM, 0);
  digitalWrite(leftMotorF, HIGH);
  digitalWrite(leftMotorB, LOW);
  analogWrite(leftMotorPWM, 100);
  digitalWrite(stby,HIGH);
}

void soft_right() {
  digitalWrite(rightMotorF, HIGH);
  digitalWrite(rightMotorB, LOW);
  analogWrite(rightMotorPWM, 50);
  digitalWrite(leftMotorF, HIGH);
  digitalWrite(leftMotorB, LOW);
  analogWrite(leftMotorPWM, 100);
  digitalWrite(stby,HIGH);
}

void sstop() {
  digitalWrite(rightMotorF, HIGH);
  digitalWrite(rightMotorB, LOW);
  analogWrite(rightMotorPWM, 0);
  digitalWrite(leftMotorF, HIGH);
  digitalWrite(leftMotorB, LOW);
  analogWrite(leftMotorPWM, 0);
  digitalWrite(stby,HIGH);
}