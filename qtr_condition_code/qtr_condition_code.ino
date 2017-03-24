#include <QTRSensors.h>

#define rightMotorF 4
#define rightMotorB 5
#define rightMotorPWM 9
#define leftMotorF 7
#define leftMotorB 8
#define leftMotorPWM 3
#define stby 6

QTRSensorsRC qtr((unsigned char[]) {
  A0, 12, A1, A2, A3, A4, 11, A5
}, 8, 2500);

void setup()
{
  pinMode(rightMotorF, OUTPUT);
  pinMode(rightMotorB, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotorF, OUTPUT);
  pinMode(leftMotorB, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(stby, OUTPUT);
  for (int i = 0; i < 100; i++)  // make the calibration take about 5 seconds
  {
    qtr.calibrate();
    delay(20);
  }
}

int lastError = 0;
float kp = 0.1;  // 0.08 // for small = 0.1
float kd = 5; // 1.0   // for small = 1.7
float ki = 0;
int integral = 0;
int derivative = 0;

void loop()
{
  unsigned int sensors[8];

  qtr.readCalibrated(sensors, QTR_EMITTERS_ON);

  if (sensors[0] < 600 && sensors[7] < 600) {
    if (sensors[1] < 600 && sensors[6] < 600) {
      if (sensors[2] > 600 && sensors[5] < 600) {
        move(60, 80);
      }
      else if (sensors[2] < 600 && sensors[5] > 600) {
        //got right slow
        move(80, 60);
      }
    }
    else if (sensors[1] > 600 && sensors[6] < 600) {
      //go left
      move(40, 80);
    }
    else if (sensors[1] < 600 && sensors[6] > 600) {
      //got right
      move(80, 40);
    }
    
  }
  else if (sensors[0] < 600 && sensors[7] > 600) {
    //right found
    move(40, 0);
    delay(200);
  }
  else if (sensors[0] > 600 && sensors[7] < 600) {
    //left found
    move(0, 40);
    delay(200);
  }
  
}

void move(int valueL, int valueR) {
  digitalWrite(rightMotorF, HIGH);
  digitalWrite(rightMotorB, LOW);
  analogWrite(rightMotorPWM, valueR);
  digitalWrite(leftMotorF, HIGH);
  digitalWrite(leftMotorB, LOW);
  analogWrite(leftMotorPWM, valueL);
  digitalWrite(stby, HIGH);
}
