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
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low
int maximum ;

// Create instance of sensors. Sensors connected to pins 5 to 12
//QTRSensorsRC qtrrc((unsigned char[]) {A2,A3,A4,A5},  NUM_SENSORS, TIMEOUT); //QTR_NO_EMITTER_PIN); 
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
 maximum = 130;
  followsegment();
//delay(500);
}
void followsegment(){
   
 
int position = qtrrc.readLine(sensorValues); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
/*
Serial.print("position");
Serial.print("\t");
Serial.println("power_difference");
Serial.println(position);
*/
 int proportional = int(position) - 3500;

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
 
 
    float power_difference = proportional*1/3+ integral/10000 + derivative*10;
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
if(( sensorValues[3] > 300 && sensorValues[4] >300  )&& (sensorValues[0]>300 || sensorValues[7] >300)) 

    {
     //maze();
       stop1();
       delay(3000);
      // maximum=(maximum-90);                                
    }
 
/*if( (sensorValues[0]<50 && sensorValues[1] <50 &&sensorValues[2]<50 && sensorValues[3]<50 &&sensorValues[4] <50 &&sensorValues[5] <50) &&(sensor[0] <50&&sensor[1]<50)) //white
    { 
      
      backward();
       
    }
if(( sensorValues[1] >80 &&sensorValues[2] >80 && sensorValues[3] >80 &&sensorValues[4] >80 )&& (sensorValues[0]>80 || sensorValues[5] >80)) 

    {
     maze();
                                       
    }*/
   
  }
void left1()
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
  analogWrite(rightMotorPWM,75);
    digitalWrite(leftMotorF,HIGH);
  digitalWrite(leftMotorB, LOW);
  analogWrite(leftMotorPWM, 75);
  digitalWrite(stby,HIGH);
  }
  void reverse()
  {
  digitalWrite(rightMotorF,LOW);
  digitalWrite(rightMotorB,HIGH);
  analogWrite(rightMotorPWM,75);
    digitalWrite(leftMotorF,LOW);
  digitalWrite(leftMotorB, HIGH);
  analogWrite(leftMotorPWM, 75);
  digitalWrite(stby,HIGH);
  }
  void right1()
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
  delay(500);

    }
    void maze()
 {    
      qtrrc.readLine(sensorValues);
   
  if(sensorValues[0]>300 && sensorValues[4]>300&&sensorValues[5]>300 )  //left
  {
     /*reverse();
     delay(275);
    stop1();
    delay(3000);     
    left1();
     delay(250);*/
        
  }  
  else
  if(sensorValues[7]>300 && sensorValues[3]>300 &&sensorValues[4]>300 )// right
     {
      reverse();
     delay(275);
       
    stop1();
    delay(3000);
       right1();
              delay(250);
             
       
        }
       else
  if(sensorValues[0]>300 &&sensorValues[1]>300&& sensorValues[2]>300&& sensorValues[3]>300 && sensorValues[4]>300&&sensorValues[5]>300&&sensorValues[6]>300&&sensorValues[7]>300)// WHITE GAPS
    {
      straight();
    delay(4000);
      }    
 /* if(sensorValues[0]<300 &&sensorValues[1]<300&& sensorValues[2]<300&& sensorValues[3]<300 && sensorValues[4]<300 && sensorValues[5]<300 && sensorValues[6]<300&& && sensorValues[7]<300  )// all black
    {straight();
    delay(4000);
      }*/    

/*else
     if(sensorValues[0]>80 && sensorValues[5]>80 && sensor[0]<50 && sensor[1]<50) //tpoiont
     {
        straight();
     delay(275);
      
                left();
                delay(250);
                
      }
   else
   if(sensorValues[0]>80 && sensorValues[5]>80 && sensor[0]>80 && sensor[1]>80)   //plus-point
  { 
     straight();
     delay(275);
      
    left();
   delay(250);
    
  }
else
       if(sensorValues[0]<50 && sensorValues[5]>80 && sensor[0]>80 && sensor[1]>80 )  //st-right
        {
      
          straight();
        delay(275);
          
           }
    else
         if(sensorValues[0]>80 && sensorValues[5]<50 && sensor[0]>80 && sensor[1]>80)  //straight-left
    { 
     // stop1();
      //delay(800);
    straight();
     delay(275);

//stop1();
  //    delay(800);
      
      left();
     delay(250);
//stop1();
  //    delay(800);

     }*/
    
 
 }
  
