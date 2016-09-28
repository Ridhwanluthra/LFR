#include <QTRSensors.h>
//#define QTR_NO_EMITTERS_PIN 2
#define rightMotorF 5
#define rightMotorB 4
#define rightMotorPWM 3
#define leftMotorF 8
#define leftMotorB 7
#define leftMotorPWM 9
#define stby 6
// This example is designed for use with a QTR-8RC sensor 
// This is a much cut-down version of the Pololu Example code

#define NUM_SENSORS 8    // number of sensors used
//#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low

// Create instance of sensors. Sensors connected to pins 5 to 12
//QTRSensorsRC qtrrc((unsigned char[]) {A2,A3,A4,A5},  NUM_SENSORS, TIMEOUT); //QTR_NO_EMITTER_PIN); 
QTRSensorsRC qtrrc((unsigned char[]) {A0,A1,A2,A3,A4,A5,10,11},  NUM_SENSORS); //QTR_NO_EMITTER_PIN); 

unsigned int sensorValues[NUM_SENSORS];
unsigned int last_proportional = 0;
long integral = 0;
void lefft()
{
analogWrite(leftMotorPWM,35);
analogWrite(rightMotorPWM,35);
  digitalWrite(leftMotorB,HIGH);
digitalWrite(leftMotorF,LOW);
digitalWrite(rightMotorB,LOW);
digitalWrite(rightMotorF,HIGH);
 digitalWrite(stby,HIGH);
}
void right()
{
analogWrite(leftMotorPWM,35);
analogWrite(rightMotorPWM,35);
  digitalWrite(leftMotorF,HIGH);
digitalWrite(leftMotorB,LOW);
digitalWrite(rightMotorF,LOW);
digitalWrite(rightMotorB,HIGH);
 digitalWrite(stby,HIGH);
}

void setup()
{
 // pinMode(2,INPUT);
  pinMode(10, INPUT);
  pinMode(11, INPUT);
  pinMode(rightMotorF, OUTPUT);
  pinMode(rightMotorB, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotorF, OUTPUT);
  pinMode(leftMotorB, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(stby,OUTPUT);
  
 // Serial.begin(9600);
  int k;
 //delay(200);
//for (k = 0; k < 150; k++) // make the calibration take about 5 seconds
//{
for(int counter=0;counter<60;counter++)
{qtrrc.calibrate();
//Serial.println(counter);
/*
if(counter < 15 || counter >=45)
  lefft(); // put your main code here, to run repeatedly:
else
  right(); // put your main code here, to run repeatedly:
delay(20);//}
*/
delay(30);
//Serial.println(k);
}
delay(20);
}

void loop()
{
//delay(500);
int position = qtrrc.readLine(sensorValues); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
/*
Serial.print("position");
Serial.print("\t");
Serial.println("power_difference");
Serial.println(position);
*/
 int proportional = int(position) - 3500;

 const int maximum = 150;
/*if(position>2700&&position<4150)
{maximum= maximum;
  }
  else 
  {
    maximum=(maximum-80);
    }*/
 int derivative = proportional-last_proportional;
 integral+=proportional ;
last_proportional = proportional;
 
 
    float power_difference = proportional*1/8+ integral/10000 + derivative*1.5;
/*
Serial.print("\t");
Serial.print("\t");
Serial.println(power_difference);*/
  
  if (power_difference > maximum)
    power_difference = maximum;
  if (power_difference < -maximum)
    power_difference = -maximum;

//Motor Motion
  if (power_difference < 0)//LEFT 
  {
 digitalWrite(rightMotorF, HIGH);
  digitalWrite(rightMotorB, LOW);
  analogWrite(rightMotorPWM, maximum);
    digitalWrite(leftMotorF, HIGH);
  digitalWrite(leftMotorB, LOW);
  analogWrite(leftMotorPWM, maximum+ power_difference );
  digitalWrite(stby,HIGH);
  }
  else //RIGHT
{ 
 digitalWrite(rightMotorF, HIGH);
  digitalWrite(rightMotorB, LOW);
  analogWrite(rightMotorPWM,maximum- power_difference );
    digitalWrite(leftMotorF, HIGH);
  digitalWrite(leftMotorB, LOW);
  analogWrite(leftMotorPWM, maximum);
  digitalWrite(stby,HIGH);
}

qtrrc.readCalibrated(sensorValues);
if(sensorValues[0]<300&&sensorValues[1]<300&&sensorValues[2]<300&&sensorValues[3]<300&&sensorValues[4]<300&&sensorValues[5]<300&&sensorValues[6]<300&&sensorValues[7]<300)
{//right
  //delay(50);
  digitalWrite(rightMotorF, LOW);
  digitalWrite(rightMotorB, HIGH);
  analogWrite(rightMotorPWM,maximum);
    digitalWrite(leftMotorF, HIGH);
  digitalWrite(leftMotorB, LOW);
  analogWrite(leftMotorPWM,maximum);
  digitalWrite(stby,HIGH);
  
}}/*
if(sensorValues[0]>300&&sensorValues[1]>300&&sensorValues[2]>300&&sensorValues[3]>300&&sensorValues[4]>300&&sensorValues[5]>300&&sensorValues[6]<300&&sensorValues[7]<300)
{//left
  digitalWrite(rightMotorF, HIGH);

  digitalWrite(rightMotorB, LOW);
  analogWrite(rightMotorPWM,150);
    digitalWrite(leftMotorF, LOW);
  digitalWrite(leftMotorB, HIGH);
  analogWrite(leftMotorPWM,150);
  digitalWrite(stby,HIGH);
  delay(30);
}*/
/*if(sensorValues[1]>300&&sensorValues[2]>300&&sensorValues[3]>300&&sensorValues[4]>300&&sensorValues[5]>300&&sensorValues[6]>300)
{//left
  digitalWrite(rightMotorF, HIGH);
  digitalWrite(rightMotorB, LOW);
  analogWrite(rightMotorPWM,160);
    digitalWrite(leftMotorF, HIGH);
  digitalWrite(leftMotorB, LOW);
  analogWrite(leftMotorPWM,160);
  digitalWrite(stby,HIGH);
  delay(50);
}*/





