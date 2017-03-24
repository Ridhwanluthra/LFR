#include <QTRSensors.h>

#define rightMotorF 7
#define rightMotorB 8
#define rightMotorPWM 9
#define leftMotorF 4
#define leftMotorB 5
#define leftMotorPWM 3
#define rightSensor A6
#define forwardSensor 2
#define stby 6
#define led 13

// QTRSensorsRC qtr((unsigned char[]) {A0,11, A1, A2, A3, A4, 12, A5}, 8, 2500);
QTRSensorsRC qtr((unsigned char[]) {A5, 12, A4, A3, A2, A1, 11, A0}, 8, 2500);
// QTRSensorsRC qtr((unsigned char[]) {A0, 12, A2, A3, A4, A5, 11, A1}, 8, 2500);

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
  // Serial.begin(9600);
 
}

unsigned int sensors[8];

//=====Constants for Line Following====================================================
#define kp 0.05
#define kd 4
#define ki 0
//==============xxxx=========xxxx=============xxxx===================================

int countfinal=0;
//======Constants for wall following===================================================
int x;
int y;
bool wall_sure = false;
bool wall_found = false;

#define kpW 0.5
#define kdW 0
#define kiW 0
//============xxxx======xxxx===========xxxx============================================
int count = 0;

#define set_distance 150

void loop() {
  x = analogRead(rightSensor);
  y = digitalRead(forwardSensor);

  if(x > set_distance) {
    count++;
  }
  else {
    count = 0;
  }
  if(count > 10)
  {
    wall_sure = true;
  }

  if (!wall_sure) {
    left_line_follow(120);
  }
  else {
    wall_follow(70);
    if (x < set_distance && (sensors[0] > 700 || sensors[1] > 700 || sensors[2] > 700 || sensors[3] > 700 || sensors[4] > 700 || sensors[5] > 700 || sensors[6] > 700 || sensors[7] > 700)) {
      wall_sure = false;
    }
  }
}

void simple_case() {
  static int j;
  if(y == 0) {
    j++;
  }
  else {
    j = 0;
    wall_found = false;
  }
  if(j > 5)
  {
    wall_found = true;
  }

  if(wall_found) {
    digitalWrite(rightMotorF, HIGH);
    digitalWrite(rightMotorB, LOW);
    analogWrite(rightMotorPWM,100);

    digitalWrite(leftMotorF, LOW);
    digitalWrite(leftMotorB, HIGH);
    analogWrite(leftMotorPWM, 100);
    digitalWrite(stby,HIGH);
    delay(200);
  }
}

void wall_follow(int maximum) {
  static int lastErrorW;
  static int integralW;

  // simple_case();
  
  int errorW = int(x) - 170;
  
  integralW += errorW;
  int derivativeW = errorW - lastErrorW;
  int power_differenceW = kpW * errorW + kiW * integralW + kdW * derivativeW;
  lastErrorW = errorW;

  to_motors(-power_differenceW, maximum);
}


void line_follow(int maximum) {
  static int lastError;
  static int integral;
  unsigned int position = qtr.readLine(sensors, QTR_EMITTERS_ON,0);

  int error = int(position) - 3500;
  
  integral += error;
  int derivative = error - lastError;
  int power_difference = kp * error + ki * integral + kd * derivative;
  lastError = error;
  
  to_motors(power_difference, maximum);
}

void left_line_follow(int maximum) {
  static int lastError;
  static int integral;
  unsigned int position = qtr.readLine(sensors, QTR_EMITTERS_ON,0);

  int error = int(position) - 3500;
  
  integral += error;
  int derivative = error - lastError;
  int power_difference = kp * error + ki * integral + kd * derivative;
  lastError = error;

  bool u_turn;
  static int iter_count = 0;
  if (sensors[0] < 200 && sensors[1] < 200 && sensors[2] < 200 && sensors[3] < 200 && sensors[4] < 200 && sensors[5] < 200 && sensors[6] < 200 && sensors[7] < 200) {
    u_turn = true;
    iter_count = 0;
  }
  if (iter_count > 60) {
    u_turn = false;
  }
  iter_count++;
  if (!u_turn){
    check_turn_left();
    for(int i = 0; i < 8; i++) {
      sensors[i] = 0;
    }
    sensors[7] = 1000;
  }
  
  to_motors(power_difference, maximum);
}

void check_turn_left() {
  if (sensors[0] > 700 && sensors[1] > 700 && sensors[2] > 700 && sensors[3] > 700 && sensors[4] > 700) {
    digitalWrite(rightMotorF, HIGH);
    digitalWrite(rightMotorB, LOW);
    analogWrite(rightMotorPWM, 0);
    digitalWrite(leftMotorF, HIGH);
    digitalWrite(leftMotorB, LOW);
    analogWrite(leftMotorPWM, 0);
    digitalWrite(stby,HIGH);
    delay(30);

    digitalWrite(rightMotorF, HIGH);
    digitalWrite(rightMotorB, LOW);
    analogWrite(rightMotorPWM, 140);
    digitalWrite(leftMotorF, HIGH);
    digitalWrite(leftMotorB, LOW);
    analogWrite(leftMotorPWM, 0);
    digitalWrite(stby,HIGH);
    delay(300);
  }
}

// void delay_line_follow(int maximum) {
//   static int lastError;
//   static int integral;
//   int i = 0;
//   while (1) {
//     unsigned int position = qtr.readLine(sensors, QTR_EMITTERS_ON,0);

//     int error = int(position) - 3500;
    
//     integral += error;
//     int derivative = error - lastError;
//     int power_difference = kp * error + ki * integral + kd * derivative;
//     lastError = error;
    
//     if (sensors[7] > 700 && sensors[6] > 700 && sensors[5] > 700 && sensors[4] > 700 && sensors[3] > 700) {
//       turn_right();
//     }

//     to_motors(power_difference, maximum);
//     i++;
//   }
// }

void biased_line_follow(int maximum) {
  static int lastError;
  static int integral;
  bool right = false;
  for (int i = 300; i > 0; i--) {
    if (sensors[7] > 700) {
      right = true;
    }
    unsigned int position = qtr.readLine(sensors, QTR_EMITTERS_ON,0);
    if (right == true) {
      if (position <= 5000) {
        position += 2000;
      }
      else {
        position = 7000;
      }
    }
    int error = int(position) - 3500;
    
    integral += error;
    int derivative = error - lastError;
    int power_difference = kp * error + ki * integral + kd * derivative;
    lastError = error;

    // if (sensors[7] > 700) {
    //   right = true;
    // }
    // if (right == true) {
    //   to_motors(power_difference+100, maximum);
    // }
    // else {
    to_motors(power_difference, maximum);
    // }
  }
}

void to_motors(int power_difference, int maximum) {
  if (power_difference > maximum)
    power_difference = maximum;
  if (power_difference < -maximum)
    power_difference = -maximum;

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
}

void print_qtr() {
  for(int i=0;i<8;i++) {
    Serial.print(sensors[i]);
    Serial.print("\t");
  }
  Serial.println();
}