
#include <QTRSensors.h>
//#define QTR_NO_EMITTERS_PIN 2
#define rightMotorF 8
#define rightMotorB 7
#define rightMotorPWM 9
#define leftMotorF 6
#define leftMotorB 4
#define leftMotorPWM 3
#define stby 5
// This example is designed for use with a QTR-8RC sensor 
// This is a much cut-down version of the Pololu Example code

#define NUM_SENSORS  6   // number of sensors used
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low

// Create instance of sensors. Sensors connected to pins 5 to 12
QTRSensorsRC qtrrc((unsigned char[]) {A0,A1,A2,A3,A4,A5,10,11},  NUM_SENSORS, TIMEOUT); //QTR_NO_EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];
unsigned int last_proportional = 0;
long integral = 0;

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
  
  int k;
 delay(200);
for (k = 0; k < 250; k++) // make the calibration take about 5 seconds
{
qtrrc.calibrate();
delay(20);
Serial.println(k);
}
delay(2500);
}

void loop()
{
int position = qtrrc.readLine(sensorValues); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.



//Maths Part
 int proportional = int(position) - 2500;
 int derivative = proportional-last_proportional;
 integral+=proportional ;
last_proportional = proportional;
 
  
    int power_difference = proportiona*5 + integral/10 + derivative*4;

  const int maximum = 130;
  if (power_difference > maximum)
    power_difference = maximum;
  if (power_difference < -maximum)
    power_difference = -maximum;

//Motor Motion
  if (power_difference < 0)
  {
 digitalWrite(rightMotorF, HIGH);
  digitalWrite(rightMotorB, LOW);
  analogWrite(rightMotorPWM, maximum);
    digitalWrite(leftMotorF, HIGH);
  digitalWrite(leftMotorB, LOW);
  analogWrite(leftMotorPWM, maximum + power_difference);
  digitalWrite(stby,HIGH);
  }
  else 
{ 
 digitalWrite(rightMotorF, HIGH);
  digitalWrite(rightMotorB, LOW);
  analogWrite(rightMotorPWM,maximum - power_difference);
    digitalWrite(leftMotorF, HIGH);
  digitalWrite(leftMotorB, LOW);
  analogWrite(leftMotorPWM, maximum);
  digitalWrite(stby,HIGH);
}
}

}
