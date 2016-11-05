
#define rightMotorF 4
#define rightMotorB 5
#define rightMotorPWM 3
#define leftMotorF 7
#define leftMotorB 8
#define leftMotorPWM 9
#define stby 6


void setup() {
pinMode(rightMotorF, OUTPUT);
  pinMode(rightMotorB, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotorF, OUTPUT);
  pinMode(leftMotorB, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(stby,OUTPUT);
  
}

void loop() {
  //motor motion
  digitalWrite(rightMotorF, HIGH);
  digitalWrite(rightMotorB, LOW);
  analogWrite(rightMotorPWM,100);

  
   digitalWrite(leftMotorF, HIGH);
  digitalWrite(leftMotorB, LOW);
  analogWrite(leftMotorPWM, 100);
  digitalWrite(stby,HIGH);
  

}
