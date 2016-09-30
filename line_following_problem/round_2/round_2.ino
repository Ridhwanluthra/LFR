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
float kd = 1.7; // 1.0   // for small = 1.7
float ki = 0;
int integral = 0;
int derivative = 0;
 
void loop()
{
  unsigned int sensors[8];
  
  int position = qtr.readLine(sensors);
  
  if (sensors[0] < 200 && sensors[1] < 200  && sensors[2] < 200 && sensors[3] < 200 && sensors[4] < 200 && sensors[5] < 200 && sensors[6] < 200 && sensors[7] < 200) {
    forward();
  }
  else if(sensors[0] > 200 && sensors[1] >200  && sensors[2]>200 && sensors[3] >200 && sensors[4] > 200 && sensors[5] > 200 && sensors[6] > 200 && sensors[7] > 200) {
    forward();
    delay(90);
    stop1();
    delay(2000);
    right();
    delay(300);
  }
  else {
//      for (unsigned char i = 0; i < 6; i++)
//    {
//      Serial.print(sensors[i]);
//      Serial.print('\t');
//    }
//    Serial.println();
    
    int error = int(position) - 3500;
    integral += error;
    derivative = error - lastError;
    int power_difference = kp * error + ki * integral + kd * derivative;
    //Serial.println(power_difference);
    lastError = error;
    
    const int maximum = 80;
    
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
}

void stop1() {
  digitalWrite(rightMotorF,LOW);
  digitalWrite(rightMotorB, LOW);
  analogWrite(rightMotorPWM,0 );
  digitalWrite(leftMotorF, LOW);
  digitalWrite(leftMotorB, LOW);
  analogWrite(leftMotorPWM, 0);
  digitalWrite(stby,LOW);
}

void right()
  {    
   digitalWrite(rightMotorF,LOW);
  digitalWrite(rightMotorB, HIGH);
  analogWrite(rightMotorPWM,75);
    digitalWrite(leftMotorF, HIGH);
  digitalWrite(leftMotorB, LOW);
  analogWrite(leftMotorPWM, 75);
  digitalWrite(stby,HIGH);
   }
void forward()
  {
  digitalWrite(rightMotorF,HIGH);
  digitalWrite(rightMotorB,LOW);
  analogWrite(rightMotorPWM,75);
    digitalWrite(leftMotorF,HIGH);
  digitalWrite(leftMotorB, LOW);
  analogWrite(leftMotorPWM, 75);
  digitalWrite(stby,HIGH);
  }

