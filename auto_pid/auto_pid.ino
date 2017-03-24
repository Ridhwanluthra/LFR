#include <QTRSensors.h>

#define rightMotorF 6
#define rightMotorB 7
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
  for (int i = 0; i < 250; i++)  // make the calibration take about 5 seconds
  {
    qtr.calibrate();
    delay(20);
  }
}

int param[3] = {0.1,0,0};
int diff_param[3] = {0.1,1,1};
int last_proportional = 0;
int proportional = 0;
int integral = 0;
int derivative = 0;
int max_error = 0, new_error = 0;
bool success = false;

void loop() {
  autotune();
}

void autotune() {
  for (int i = 0; i < 3; i++) {
    param[i] += diff_param[i];
    new_error = check_error(param);
    
    if (new_error < max_error) {
      max_error = new_error;
      diff_param[i] *= 1.1;
      success = true;
    }
    if (success == false) {
      param[i] -= 2 * diff_param[i];
      new_error = check_error(param);
      if (new_error < max_error) {
        max_error = new_error;
        diff_param[i] *= 1.1;
        success = true;
      }
    }
    if (success == false) {
      param[i] += diff_param[i];
      diff_param[i] *= 0.9;
    }
    success = false;  }
}


int check_error(int* param) {
  for(int i = 0; i<500; i++) {
    unsigned int sensors[8];
    
    int position = qtr.readLine(sensors);
   
    proportional = int(position) - 3500;
    integral += proportional;
    derivative = proportional - last_proportional;
    
    int power_difference = param[0] * proportional + param[1] * integral + param[2] * derivative;
    //Serial.println(power_difference);
    last_proportional = proportional;
    
    const int maximum = 75;
    
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
