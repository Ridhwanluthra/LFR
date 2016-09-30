#include <QTRSensors.h>

#define rightMotor2 7
#define rightMotor1 6
#define rightMotorPWM 5
#define leftMotor2 10
#define leftMotor1 9
#define leftMotorPWM 11
#define stby 8

QTRSensorsRC qtr((unsigned char[]) {A0,4, A1, A2, A3, A4, 2, A5}, 8, 2500);
 
void setup()
{
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(stby,OUTPUT);
  for (int i = 0; i < 250; i++)  // make the calibration take about 5 seconds
  {
    qtr.calibrate();
    delay(20);
  }
}

void loop()
{
  unsigned int sensors[8];
  int position = qtr.readLine(sensors);
 
  // compute our "error" from the line position.  We will make it so that the error is zero 
  // when the middle sensor is over the line, because this is our goal.  Error will range from
  // -1000 to +1000.  If we have sensor 0 on the left and sensor 2 on the right,  a reading of 
  // -1000 means that we see the line on the left and a reading of +1000 means we see the 
  // line on the right.
  int error = position - 3500;
 
  int leftMotorSpeed = 75;
  int rightMotorSpeed = 75;
  if (error < -500)  // the line is on the left
    leftMotorSpeed = 0;  // turn left
  if (error > 500)  // the line is on the right
    rightMotorSpeed = 0;  // turn right

  digitalWrite(rightMotor2, HIGH);
  digitalWrite(rightMotor1, LOW);
  analogWrite(rightMotorPWM,rightMotorSpeed);

  digitalWrite(leftMotor2, HIGH);
  digitalWrite(leftMotor1, LOW);
  analogWrite(leftMotorPWM, leftMotorSpeed);
  digitalWrite(stby,HIGH);
}
