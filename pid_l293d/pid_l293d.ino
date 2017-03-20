#include <QTRSensors.h>

#define rightMotorF 5
#define rightMotorB 4

#define leftMotorF 8
#define leftMotorB 7

QTRSensorsRC qtr((unsigned char[]) {A0,12, A1, A2, A3, A4, 11, A5}, 8, 2500);
 
void setup()
{
  pinMode(rightMotorF, OUTPUT);
  pinMode(rightMotorB, OUTPUT);
  pinMode(leftMotorF, OUTPUT);
  pinMode(leftMotorB, OUTPUT);

  for (int i = 0; i < 100; i++)  // make the calibration take about 5 seconds
  {
    qtr.calibrate();
    delay(20);
  }
}

int lastError = 0;
float kp = 0.1;  // 0.08 // for small = 0.1
float kd = 5; // 1.0   // for small = 1.7
float ki = 0;
int integral = 0;
int derivative = 0;
 
void loop()
{
  unsigned int sensors[8];
  
  unsigned int position = qtr.readLine(sensors, QTR_EMITTERS_ON);

 int error = int(position) - 3500;
  integral += error;
  derivative = error - lastError;
digitalWrite(leftMotorB, LOW);  int power_difference = kp * error + ki * integral + kd * derivative;
  lastError = error;
  
  const int maximum = 100;
  
  if (power_difference > maximum)
    power_difference = maximum;
  if (power_difference < -maximum)
    power_difference = -maximum;  
    
  if (power_difference < 0) {
    analogWrite(rightMotorF, maximum);
    analogWrite(leftMotorF, maximum + power_difference);

    digitalWrite(rightMotorB, LOW);
    digitalWrite(leftMotorB, LOW);
  } 
  else { 
    analogWrite(rightMotorF, maximum - power_difference);
    analogWrite(leftMotorF, maximum);

    digitalWrite(leftMotorB, LOW);
    digitalWrite(rightMotorB, LOW);
  }
}

