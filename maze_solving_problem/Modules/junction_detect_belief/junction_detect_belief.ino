#include <QTRSensors.h>

#define rightMotorF 7
#define rightMotorB 6
#define rightMotorPWM 5
#define leftMotorF 10
#define leftMotorB 9
#define leftMotorPWM 11
#define stby 8

QTRSensorsRC qtr((unsigned char[]) {A0,4, A1, A2, A3, A4, 2, A5}, 8, 2500);
 
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
float kp = 0.1;  // 0.08 // for small = 0.1
float kd = 8; // 1.0   // for small = 1.7
float ki = 0;
int integral = 0;
int derivative = 0;
 
void loop()
{
  unsigned int sensors[8];
  
  int position = qtr.readLine(sensors,QTR_EMITTERS_ON,1);
 
  int error = int(position) - 3500;
  integral += error;
  derivative = error - lastError;
  int power_difference = kp * error + ki * integral + kd * derivative;
  Serial.println(power_difference);
  lastError = error;
  
  const int maximum = 200;
  
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

int check_junction()
{
  //for 90L - belief state 1

  if(sensors[0]<200 && sensors[5]>800) {
  
}
  //for 90R - belief state 2

  if(sensors[0]>800 && sensors[5]<200) {
    
  }

  //for T(2) i.e. 3 and Y(2) i.e. 6
  //for y(2) sensor[0] and [7] might not read white always
  //Belief state - 3
  if(sensors[0]<800 && sensors[1]<200 && sensors[2]<200 && sensors[3]<200 && sensors[4]<200 && sensors[5]<800) {
    
  }

}
