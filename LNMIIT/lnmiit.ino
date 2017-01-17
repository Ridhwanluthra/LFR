#include <QTRSensors.h>

#define rightMotorF 4
#define rightMotorB 5
#define rightMotorPWM 9
#define leftMotorF 7
#define leftMotorB 8
#define leftMotorPWM 3
#define stby 6

QTRSensorsRC qtr((unsigned char[]) {A0,11, A1, A2, A3, A4, 12, A5}, 8, 2500);
 
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
  Serial.begin(9600);
}

int lastError = 0;
float kp = 0.05;  // 0.08 // for small = 0.1
float kd = 2; // 1.0   // for small = 1.7
float ki = 0;
int integral = 0;
int derivative = 0;
 
void loop()
{
  unsigned int sensors[8];
  const int maxCount = 10;
  unsigned int position = qtr.readLine(sensors, QTR_EMITTERS_ON,1);
 /*for(int i=0;i<8;i++)
 {
   Serial.print(sensors[i]);
   Serial.print("\t");
 }
 */
  int error = int(position) - 3500;
  static int iterCount = 0; 
  int leftIter[maxCount];
  int rightIter[maxCount];
  iterCount %= maxCount;           //Ensures iteration count remains within range
  leftIter[iterCount] = sensors[0];
  rightIter[iterCount] = sensors[7];
  //Find a less interfering method to do the following
  bool acuteFlag = true;
  for (int i = 0; i < maxCount; i++) {
    if (sensors[i]) {
      acuteFlag = false;
      break;
    }
  }
  enum NextTurn {left, right, none};
  NextTurn nextTurn = none;
  if (acuteFlag) {
    for (int i = 0; i < maxCount; i++) {
      if (leftIter[i]) {
        nextTurn = left;
        break;
      }
      if (rightIter[i]) {
        nextTurn = right;
      }
    }
  }
  switch (nextTurn) {
    case none:
      break;
    case left:
      //Turn left
    case right:
      //Turn right
    default:
  }
  integral += error;
  derivative = error - lastError;
  int power_difference = kp * error + ki * integral + kd * derivative;
  lastError = error;
  
  const int maximum = 100;
  
  if (power_difference > maximum)
    power_difference = maximum;
  if (power_difference < -maximum)
    power_difference = -maximum;  
  //Serial.print(power_difference);
  //Serial.println("");
  if (power_difference >0) {
    digitalWrite(rightMotorF, HIGH);
    digitalWrite(rightMotorB, LOW);
    analogWrite(rightMotorPWM, maximum - power_difference);
    digitalWrite(leftMotorF, HIGH);
    digitalWrite(leftMotorB, LOW);
    analogWrite(leftMotorPWM, maximum);
    digitalWrite(stby,HIGH);
  } 
  else { 
    digitalWrite(rightMotorF, HIGH);
    digitalWrite(rightMotorB, LOW);
    analogWrite(rightMotorPWM, maximum);
    digitalWrite(leftMotorF, HIGH);
    digitalWrite(leftMotorB, LOW);
    analogWrite(leftMotorPWM, maximum + power_difference);
    digitalWrite(stby,HIGH);
  }
  /*
  if(sensors[0]<300)
  
  {
     digitalWrite(rightMotorF, HIGH);
    digitalWrite(rightMotorB, LOW);
    analogWrite(rightMotorPWM, 100);
    digitalWrite(leftMotorF, LOW);
    digitalWrite(leftMotorB, LOW);
    analogWrite(leftMotorPWM, 0);
    delay(500);
}
 if(sensors[7]<300)
  
  {
     digitalWrite(rightMotorF, LOW);
    digitalWrite(rightMotorB, LOW);
    analogWrite(rightMotorPWM, 0);
    digitalWrite(leftMotorF, HIGH);
    digitalWrite(leftMotorB, LOW);
    analogWrite(leftMotorPWM, 100);
    delay(500);
}*/
}
