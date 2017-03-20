
#define rightMotor2 4
#define rightMotor1 5
#define rightMotorPWM 9
#define leftMotor2 7
#define leftMotor1 8
#define leftMotorPWM 3
#define stby 6


void setup() {
pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(stby,OUTPUT);
  
}

void loop() {
  //motor motion
  digitalWrite(rightMotor2, HIGH);
  digitalWrite(rightMotor1, LOW);
  analogWrite(rightMotorPWM,50);

  
   digitalWrite(leftMotor2, HIGH);
  digitalWrite(leftMotor1, LOW);
  analogWrite(leftMotorPWM, 50);
  digitalWrite(stby,HIGH);
  

}
