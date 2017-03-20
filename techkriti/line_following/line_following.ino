#include <QTRSensors.h>

#define rightMotorF 4
#define rightMotorB 5
#define rightMotorPWM 9
#define leftMotorF 7
#define leftMotorB 8
#define leftMotorPWM 3
#define stby 6

QTRSensorsRC qtr((unsigned char[]) {A0, 12, A2, A3, A4, A5, 11, A1}, 8, 2500);
 
void setup()
{
  pinMode(rightMotorF, OUTPUT);
  pinMode(rightMotorB, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotorF, OUTPUT);
  pinMode(leftMotorB, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(stby,OUTPUT);
  Serial.begin(9600);
  for (int i = 0; i < 100; i++)  // make the calibration take about 5 seconds
  {
    qtr.calibrate();
    delay(20);
  }
}

int lastError = 0;
float kp = 0.05;  // 0.08 // for small = 0.1
float kd = 1.5; // 1.0   // for small = 1.7
float ki = 0.001;
int integral = 0;
int derivative = 0;
int error, power_difference;
 
void loop()
{
  unsigned int sensors[8];
  
  int position = qtr.readLine(sensors, QTR_EMITTERS_ON, 1);
 
  error = 3500 - int(position);
  integral += error;
  derivative = error - lastError;
  power_difference = kp * error + ki * integral + kd * derivative;  
  Serial.println(power_difference);
  lastError = error;
  
  const int maximum = 100;
  
  if (power_difference > maximum)
    power_difference = maximum;
  if (power_difference < -maximum)
    power_difference = -maximum;  
    
  if (power_difference < 0) {
    digitalWrite(rightMotorF, HIGH);
    digitalWrite(rightMotorB, LOW);
    analogWrite(rightMotorPWM, maximum);
    digitalWrite(leftMotorF, HIGH);
    digitalWrite(leftMotorB, LOW);
    analogWrite(leftMotorPWM, maximum + power_difference);
    digitalWrite(stby,HIGH);
  } 
  else { 
    digitalWrite(rightMotorF, HIGH);
    digitalWrite(rightMotorB, LOW);
    analogWrite(rightMotorPWM, maximum - power_difference);
    digitalWrite(leftMotorF, HIGH);
    digitalWrite(leftMotorB, LOW);
    analogWrite(leftMotorPWM, maximum);
    digitalWrite(stby,HIGH);
  }
}

