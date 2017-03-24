#include <QTRSensors.h>

#define rightMotorF 4
#define rightMotorB 5
#define rightMotorPWM 3
#define leftMotorF 7
#define leftMotorB 8
#define leftMotorPWM 9
#define stby 6

QTRSensorsRC qtr((unsigned char[]) {A0,12, A1, A2, A3, A4, 11, A5}, 8, 2500);
 
void setup()
{
  pinMode(rightMotorF, OUTPUT);
  pinMode(rightMotorB, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotorF, OUTPUT);
  pinMode(leftMotorB, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(stby,OUTPUT);
  for (int i = 0; i < 100; i++)  // make the calibration take about 5 seconds
  {
    qtr.calibrate();
    delay(20);
  }
}

int lastError = 0;
float kp = 0.09 ;  // 0.08 // for small = 0.1
float kd = 4; // 1.0   // for small = 1.7
float ki = 0;
int integral = 0;
int derivative = 0;
 
void loop()
{
  unsigned int sensors[8];

  unsigned int position = qtr.readLine(sensors, QTR_EMITTERS_ON,1);


  //code for detecting acute angles
  int counter = 0;
  for (int i = 0; i < 8; i++) {
  	//please check if > 500 is white 
    if (sensor[i] > 500) //white found
    {
      if (counter == 0 || counter == 2) {
        counter++;
      }
    }
    //please check if < 500 is black
    else if (sensor[i] < 500) // black found
    {
      if (counter == 1) {
        counter++;
      }
    }
    
    if (counter = 3) {
      if (i > 5) {
        // turn right
        right();
        delay(1000);
        break;
      }
      else {
        //turn left
        left();
        delay(1000);
        break;
      }
    }
  }




/*
  if(sensors[0]<500)
  {
  	left();
    delay(1000);
  }
  else if(sensors[7]<500)
  {
    right();
    delay(1000);
  }
 */
  int error = int(position) - 3500;
  integral += error;
  derivative = error - lastError;
  int power_difference = kp * error + ki * integral + kd * derivative;
  lastError = error;
  
  const int maximum = 90;
  
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

void left() {
	digitalWrite(rightMotorF, HIGH);
    digitalWrite(rightMotorB, LOW);
    analogWrite(rightMotorPWM, 60);
    digitalWrite(leftMotorF, LOW);
    digitalWrite(leftMotorB, LOW);
    analogWrite(leftMotorPWM, 60);
    digitalWrite(stby,HIGH);
}

void right() {
	digitalWrite(rightMotorF, LOW);
    digitalWrite(rightMotorB, LOW);
    analogWrite(rightMotorPWM, 60);
    digitalWrite(leftMotorF, HIGH);
    digitalWrite(leftMotorB, LOW);
    analogWrite(leftMotorPWM, 60);
    digitalWrite(stby,HIGH);
}

