#include <QTRSensors.h>
#define rightMotorF 4
#define rightMotorB 5
#define rightMotorPWM 3
#define leftMotorF 8
#define leftMotorB 7
#define leftMotorPWM 9
#define stby 6

#define NUM_SENSORS  6   // number of sensors used
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low
#define Tnum_sensor 2
 int k;
QTRSensorsRC qtrrc((unsigned char[]) {A0,A1,A2,A3,A4,A5},  NUM_SENSORS, TIMEOUT); //QTR_NO_EMITTER_PIN); 
QTRSensorsRC qtrrc1((unsigned char[]) {10,11}, Tnum_sensor, TIMEOUT); //QTR_NO_EMITTER_PIN); 

unsigned int sensorValues[NUM_SENSORS];
unsigned int sensor[Tnum_sensor];
unsigned int last_proportional = 0;
long integral = 0;
const int maximum = 150;
  
void setup()
{
  pinMode(10, INPUT);
  pinMode(11, INPUT);
  pinMode(rightMotorF, OUTPUT); 
  pinMode(rightMotorB, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(rightMotorF, OUTPUT);
  pinMode(rightMotorB, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(stby,OUTPUT);
  
  Serial.begin(9600);
  
 delay(200);
for (k = 0; k < 150; k++) // make the calibration take about 5 seconds
{
qtrrc.calibrate();
qtrrc1.calibrate();
delay(20);
Serial.println(k);
}
delay(20);
}

void loop()
{
//qtrrc1.readLine(sensor);

int position = qtrrc.readLine(sensorValues); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
//Serial.println(position);

//Maths Part
 int proportional = 2500-int(position);
 int derivative = proportional-last_proportional;
 integral+=proportional ;
last_proportional = proportional;
 
  
    int power_difference = proportional*1/2 + integral/10000 + derivative*8;

  if (power_difference > maximum)
    power_difference = maximum;
  if (power_difference < -maximum)
    power_difference = -maximum;

//Motor Motion
  if (power_difference < 0)//left 
  {
 digitalWrite(rightMotorF, HIGH);
  digitalWrite(rightMotorB, LOW);
  analogWrite(rightMotorPWM, maximum);
    digitalWrite(leftMotorF, HIGH);
  digitalWrite(leftMotorB, LOW);
  analogWrite(leftMotorPWM, maximum + power_difference);
  digitalWrite(stby,HIGH);
  
  }
  else //right
{ 
 digitalWrite(rightMotorF, HIGH);
  digitalWrite(rightMotorB, LOW);
  analogWrite(rightMotorPWM,maximum - power_difference);
    digitalWrite(leftMotorF, HIGH);
  digitalWrite(leftMotorB, LOW);
  analogWrite(leftMotorPWM, maximum);
  digitalWrite(stby,HIGH);
  
}
qtrrc.readCalibrated(sensorValues);
qtrrc1.readCalibrated(sensor);
 
if( (sensorValues[0]>200 && sensorValues[1] >200 &&sensorValues[2]>200 && sensorValues[3]>200 &&sensorValues[4] > 200 &&sensorValues[5] > 200) &&(sensor[0] >200&&sensor[1]> 200)) //black
    { 
      stop1();
        delay(3000);
      backward();
       
    }
     if(sensorValues[0]<200||sensorValues[5]<200)
    {

     maze();
                                        
    }
}

void left()
 { 
digitalWrite(rightMotorF, HIGH);
  digitalWrite(rightMotorB,LOW);
  analogWrite(rightMotorPWM,100 );
    digitalWrite(leftMotorF,LOW);
  digitalWrite(leftMotorB, HIGH);
  analogWrite(leftMotorPWM, 100);
  digitalWrite(stby,HIGH);
 }
  void straight()
  {
  digitalWrite(rightMotorF,HIGH);
  digitalWrite(rightMotorB,LOW);
  analogWrite(rightMotorPWM,50);
    digitalWrite(leftMotorF,HIGH);
  digitalWrite(leftMotorB, LOW);
  analogWrite(leftMotorPWM, 50);
  digitalWrite(stby,HIGH);
  }
  void right()
  {    
   digitalWrite(rightMotorF,LOW);
  digitalWrite(rightMotorB, HIGH);
  analogWrite(rightMotorPWM,100 );
    digitalWrite(leftMotorF, HIGH);
  digitalWrite(leftMotorB, LOW);
  analogWrite(leftMotorPWM, 100);
  digitalWrite(stby,HIGH);
   }
   void stop1()
   {
    digitalWrite(rightMotorF,LOW);
  digitalWrite(rightMotorB, LOW);
  analogWrite(rightMotorPWM,0 );
    digitalWrite(leftMotorF, LOW);
  digitalWrite(leftMotorB, LOW);
  analogWrite(leftMotorPWM, 0);
  digitalWrite(stby,LOW);
   }
    void backward()
{
   digitalWrite(rightMotorF,HIGH);
  digitalWrite(rightMotorB, LOW);
  analogWrite(rightMotorPWM,100 );
    digitalWrite(leftMotorF, LOW);
  digitalWrite(leftMotorB, HIGH);
  analogWrite(leftMotorPWM, 100);
  digitalWrite(stby,HIGH);
  delay(550);

  
  }


    void maze()
 {    
         while(1)
  {
     straight();
     delay(225);
      qtrrc.readLine(sensorValues);
     qtrrc1.readLine(sensor);

      
       
       
       if((sensorValues[0]<200 ) && (sensor[0]<200&&sensor[1]<200))  //straight-left
    { 
   /*stop1();
   delay(5000);
   straight();
   delay(50);
   stop1();
   delay(5000);
   straight();
        delay(120);
        stop1();
        delay(5000);
        left();
        delay(220);
        stop1();
        delay(5000);
        break;
     
     //stop1();
       // delay(3000);
     
     */
     left();
     delay(250);
     break;
     }
  
    if((sensorValues[5]<200 && sensorValues[4]<200&&sensorValues[0]>200)&& (sensor[0]<200&&sensor[1]<200))   //straight-right
  
     {
        //stop1();
        //delay(3000);
        straight();
        delay(100);
        break;
        }

     if((sensorValues[0]<20 && sensorValues[5]<20 &&sensorValues[1]<20 && sensorValues[4]<20)&&(sensor[0]>500&&sensor[1]>500)) //tpoiont
     {
          //      stop1();
        //delay(3000);
                left();
                delay(250);
                break;
      }
   if( (sensorValues[0]<20  &&sensorValues[5] < 20) &&(sensor[0] <20&&sensor[1] < 20))   //plus-point
  { 
    //stop1();
       // delay(3000);
    left();
   delay(250);
    break;
  }

       if((sensorValues[5]<20 )&&(sensorValues[4]<20 )&&(sensorValues[3]<20 )&& (sensor[0]>500&&sensor[1]>500))  //right
        {
         // stop1();
        //delay(3000);
              right();
              delay(250);
              break;
        }
     if((sensorValues[0]<20 )&&(sensorValues[1]<20 )&&(sensorValues[2]<20 ) && (sensor[0]>500&&sensor[1]>500))  //left
  {
 // stop1();
   //     delay(3000);
        left();
        delay(250);
        break;
  }*/
    
  }
 }
  
