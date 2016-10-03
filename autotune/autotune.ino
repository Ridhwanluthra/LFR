#include <QTRSensors.h>

#define rightMotorF 7
#define rightMotorB 6
#define rightMotorPWM 5
#define leftMotorF 10
#define leftMotorB 9
#define leftMotorPWM 11
#define stby 8
int max_error=0;
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
  max_error=check_error();
  Serial.println("INITIAL MAX ERROR");
  Serial.println(max_error);
}
int new_error=0;
float param[3]={.1,.9,0};
float diff_param[3]={.2,.4,0};
boolean  success=false;
int counter=0;
int maxError=0;
int lastError = 0;
//float kp = 0.1;  // 0.08 // for small = 0.1
//float kd = 0.9 ; // 1.0   // for small = 1.7
//float ki = 0;
int integral = 0;
int derivative = 0;
int Time, last_time=0;
int delta_time;
void loop()
{
  autotune();

  }
  
int check_error(){
  while(counter<10){
   unsigned int sensors[8];
  
  int position = qtr.readLine(sensors, QTR_EMITTERS_ON, 1);
   
    //Serial.println(position);
    int error = int(position) - 3500;
    if(maxError<error && error>=0)
      maxError=error;
    else if(maxError<(-1)*error && error<0)
           maxError=(-1)*error;
    //Serial.println(maxError);
    integral += error;
    derivative = error - lastError;
    Time=millis();
    delta_time=Time-last_time;
    int power_difference = param[0] * error + param[1] * derivative/delta_time + param[2]*integral;
    //Serial.println(power_difference);
    lastError = error;
    last_time=Time;
    
    
    const int maximum = 100;
    
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
   
  counter++;
}
//stop1();
counter=0;
//Serial.println("MAX ERROR");
return maxError;
}

void autotune() {   //TUNING THE PID PARAMETERS USING TWIDDLE ALGORITH
  
  for (int i = 0; i < 3; i++) {
    //for(int j=0 ;j<3 ;j++)
    { //Serial.print(param[j]);
      //Serial.print(" ");
     }
     //Serial.println();
    //for(int j=0 ;j<3 ;j++)
    {// Serial.print(diff_param[j]);
      //Serial.print(" ");
     }
     //Serial.println();
    param[i] += diff_param[i];
    new_error = check_error();
    //Serial.println("NEW ERROR");
    //Serial.println(new_error);
    if (new_error < max_error) {
      max_error = new_error;
      diff_param[i] *= 1.1;
      success = true;
    }
    else{
      param[i] -= 2 * diff_param[i];
      new_error = check_error();
     // Serial.println("NEW ERROR");
   // Serial.println(new_error);
      if (new_error < max_error) {
        max_error = new_error;
        diff_param[i] *= 1.1;
        success = true;
      }
    }
    if(!success){
      //Serial.println("NOT SUCCESS");
        param[i] += diff_param[i];
      diff_param[i] *= 0.9;
  
 }
    success = false;
    //Serial.println("MAX ERROR");
    //Serial.println(max_error);
    //Serial.println(param[i]);
    //delay(500);
    //counter=0;
   }
}


/*
void stop1() {
  digitalWrite(rightMotorF,LOW);
  digitalWrite(rightMotorB, LOW);
  analogWrite(rightMotorPWM,0 );
  digitalWrite(leftMotorF, LOW);
  digitalWrite(leftMotorB, LOW);
  analogWrite(leftMotorPWM, 0);
  digitalWrite(stby,LOW);
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

*/
