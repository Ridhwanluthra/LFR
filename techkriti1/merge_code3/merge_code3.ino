#include <QTRSensors.h>

#define rightMotorF 7
#define rightMotorB 8
#define rightMotorPWM 9
#define leftMotorF 4
#define leftMotorB 5
#define leftMotorPWM 3
#define rightSensor A7
#define forwardSensor 10
#define stby 6
#define led 13

QTRSensorsRC qtr((unsigned char[]) {A0,11, A1, A2, A3, A4, 12, A5}, 8, 2500);

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
  for (int i = 0; i < 100; i++)  // make the calibration take about 5 seconds
  {
    qtr.calibrate();
    delay(20);
  }
  Serial.begin(9600);
 
}

unsigned int sensors[8];

//=====Constants for Line Following====================================================
int lastError = 0;
float kp = 0.08;  // 0.08 // for small = 0.1
float kd = 4; // 1.0   // for small = 1.7
float ki = 0;
int integral = 0;
int derivative = 0;
int z=0;
//==============xxxx=========xxxx=============xxxx===================================

int countfinal=0;
//======Constants for wall following===================================================
int x;
int y;
int count=1;
int lastcount=0;
int flag=0;

int countr=1;
int lastcountr=0;
int flagr=0;


int lastErrorW = 0;
float kpW = 0.5;  // 0.08 // for small = 0.1
float kdW =20; // 1.0   // for small = 1.7
float kiW = 0;
int integralW = 0;
int derivativeW = 0;
//============xxxx======xxxx===========xxxx============================================

void loop() {
  x = analogRead(rightSensor);
  y = digitalRead(forwardSensor);
  //Serial.println(x);
  //Serial.print(" \t ");
  //Serial.print(y);
  if(x > 200) {
    if(y==0) {
      count = lastcount + 1;
    }
    else {
      count=0;
      flag=0;
    }
  
    //Serial.print(" \t ");
    //Serial.print(count);
    //Serial.print(" \t ");
    //Serial.println(flag);
    
    if(count > 30)
    {
      flag = 1;
    }
    lastcount = count;
    
    
    int errorW = int(x) - 350;
    
    integralW += errorW;
    derivativeW = errorW - lastErrorW;
    int power_differenceW = kpW * errorW + kiW * integralW + kdW * derivativeW;
    lastErrorW = errorW;
    
    const int maximumW = 170;
    
    if (power_differenceW > maximumW)
      power_differenceW = maximumW;
    if (power_differenceW < -maximumW)
      power_differenceW = -maximumW;  
      //Serial.println(power_differenceW);
      //delay(2000);
      //Serial.println("");
    if (power_differenceW >0) {
      digitalWrite(rightMotorF, HIGH);
      digitalWrite(rightMotorB, LOW);
      analogWrite(rightMotorPWM, maximumW);
      digitalWrite(leftMotorF, HIGH);
      digitalWrite(leftMotorB, LOW);
      analogWrite(leftMotorPWM, maximumW - power_differenceW);
      digitalWrite(stby,HIGH);
    } 
    else { 
      digitalWrite(rightMotorF, HIGH);
      digitalWrite(rightMotorB, LOW);
      analogWrite(rightMotorPWM, maximumW + power_differenceW);
      digitalWrite(leftMotorF, HIGH);
      digitalWrite(leftMotorB, LOW);
      analogWrite(leftMotorPWM, maximumW);
      digitalWrite(stby,HIGH);
    }
    if(flag == 1 && x > 200) {
      //Serial.print("i am succesful");
      digitalWrite(rightMotorF, HIGH);
      digitalWrite(rightMotorB, LOW);
      analogWrite(rightMotorPWM,100);

      digitalWrite(leftMotorF, LOW);
      digitalWrite(leftMotorB, HIGH);
      analogWrite(leftMotorPWM, 100);
      digitalWrite(stby,HIGH);
      delay(200);
    }
    if(flag == 0 && x < 200) {
      digitalWrite(rightMotorF, HIGH);
      digitalWrite(rightMotorB, LOW);
      analogWrite(rightMotorPWM,0);
      
      digitalWrite(leftMotorF, HIGH);
      digitalWrite(leftMotorB, LOW);
      analogWrite(leftMotorPWM, 100);
      digitalWrite(stby,HIGH);
      delay(300);
    }
  }
  else {
    unsigned int position = qtr.readLine(sensors, QTR_EMITTERS_ON,1);
     /*for(int i=0;i<8;i++)
     {
       Serial.print(sensors[i]);
       Serial.print("\t");
     }*/
 
    int error = int(position) - 3500;
    
    integral += error;
    derivative = error - lastError;
    int power_difference = kp * error + ki * integral + kd * derivative;
    lastError = error;
    
    const int maximum = 170;
    
    
    if (power_difference > maximum)
      power_difference = maximum;
    if (power_difference < -maximum)
      power_difference = -maximum;  
    //Serial.print(power_difference);
    //Serial.println("");
    if (power_difference > 0) {
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
    if(flag == 1 && x < 200) {
      digitalWrite(rightMotorF, HIGH);
      digitalWrite(rightMotorB, LOW);
      analogWrite(rightMotorPWM,100);
      
      digitalWrite(leftMotorF, LOW);
      digitalWrite(leftMotorB, HIGH);
      analogWrite(leftMotorPWM, 100);
      digitalWrite(stby,HIGH);
      delay(200);

      countfinal=1;
    }
  }
  if(countfinal == 1 && (sensors[0] < 300 || sensors[7] < 300)) {
    digitalWrite(rightMotorF, LOW);
    digitalWrite(rightMotorB, HIGH);
    analogWrite(rightMotorPWM,100);
    
    digitalWrite(leftMotorF, HIGH);
    digitalWrite(leftMotorB, LOW);
    analogWrite(leftMotorPWM, 100);
    digitalWrite(stby,HIGH);
    delay(200);
  }
}
// 
// }
// if(countfinal=1&&x<200&&sensors[0]>700&&sensors[1]>700&&sensors[2]>700&&sensors[3]>700&&sensors[4]>700&&sensors[5]>700&&sensors[6]>700&&sensors[7]>700) 
// {
//   digitalWrite(rightMotorF, HIGH);
//   digitalWrite(rightMotorB, LOW);
//   analogWrite(rightMotorPWM,100);

  
//   digitalWrite(leftMotorF, HIGH);
//   digitalWrite(leftMotorB, LOW);
//   analogWrite(leftMotorPWM, 100);
//   digitalWrite(stby,HIGH);
// }


