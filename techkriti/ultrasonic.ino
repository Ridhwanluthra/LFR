#include <QTRSensors.h>
#include <NewPing.h>

#define rightMotorF 7
#define rightMotorB 8
#define rightMotorPWM 9
#define leftMotorF 4
#define leftMotorB 5
#define leftMotorPWM 3
#define rightSensor A7
#define forwardSensor A6
#define stby 6
#define led 13

#define num_sensors 5

#define TRIGGER_PIN  10
#define ECHO_PIN     2
#define MAX_DISTANCE 200

const int latchPin = 1;//Pin 12 connected to ST_CP of 74HC595
const int clockPin = 2;//Pin 8 connected to SH_CP of 74HC595 
const int dataPin = 0; //Pin 11 connected to DS of 74HC595 
//display 0,1,2,3,4,5,6,7,8,9,A,b,C,d,E,F
int datArray[16] = {252, 96, 218, 242, 102, 182, 190, 224, 254, 246, 238, 62, 156, 122, 158, 142};

// QTRSensorsRC qtr((unsigned char[]) {A0,11, A1, A2, A3, A4, 12, A5}, 8, 2500);
QTRSensorsRC qtr((unsigned char[]) {A5, 12, A4, A3, A2, A1, 11, A0}, 8, 2500);
// QTRSensorsRC qtr((unsigned char[]) {A0, 12, A2, A3, A4, A5, 11, A1}, 8, 2500);

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

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

  pinMode(latchPin,OUTPUT);
  pinMode(clockPin,OUTPUT);
  pinMode(dataPin,OUTPUT);
  
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
#define kd 2
#define ki 0
//==============xxxx=========xxxx=============xxxx===================================

int countfinal=0;
//======Constants for wall following===================================================
int x;
int y;
bool wall_sure = false;
bool wall_found = false;

#define kpW 0.5
#define kdW 5
#define kiW 0
//============xxxx======xxxx===========xxxx============================================
int count = 0;
int led_iter = 0;
int num_blocks = 0;
int bounce_iter = 0;

#define set_distance 1200

void loop() {
  x = sonar.ping();
  y = analogRead(forwardSensor);

  wall_follow(100);

  if(x < set_distance) {
    count++;
  }
  else {
    count = 0;
  }
  if(count > 10)
  {
    wall_sure = true;
  }
  if (y < 750) {
    wall_sure = true;
    simple_case();
  }

  

  if (!wall_sure) {
    bias_line_follow(100, true);
  }
  else {
    wall_follow(100);
    unsigned int position = qtr.readLine(sensors, QTR_EMITTERS_ON,0);
    if (sensors[0] > 500 && sensors[1] > 500 && sensors[2] > 500 && sensors[3] > 500 && sensors[4] > 500 && sensors[5] > 500) {
      digitalWrite(rightMotorF, HIGH);
      digitalWrite(rightMotorB, LOW);
      analogWrite(rightMotorPWM,100);

      digitalWrite(leftMotorF, LOW);
      digitalWrite(leftMotorB, HIGH);
      analogWrite(leftMotorPWM, 100);
      digitalWrite(stby,HIGH);
      delay(200);
      while (1) {
        line_follow(100);
        follow_block();
        if (sensors[0] < 500 && sensors[1] < 500 && sensors[2] < 500 && sensors[3] < 500 && sensors[4] < 500 && sensors[5] < 500 && sensors[6] < 500 && sensors[7] < 500) {
          digitalWrite(rightMotorF, HIGH);
          digitalWrite(rightMotorB, LOW);
          analogWrite(rightMotorPWM,100);

          digitalWrite(leftMotorF, LOW);
          digitalWrite(leftMotorB, HIGH);
          analogWrite(leftMotorPWM, 100);
          digitalWrite(stby,HIGH);
        }
        if (sensors[0] > 500 && sensors[1] > 500 && sensors[2] > 500 && sensors[3] > 500 && sensors[4] > 500 && sensors[5] > 500 && sensors[6] > 500 && sensors[7] > 500) {
          show_nums(num_blocks);
          on_led();
        }
      }
    }
    if (x > set_distance && (sensors[0] > 500 || sensors[1] > 500 || sensors[2] > 500 || sensors[3] > 500 || sensors[4] > 500 || sensors[5] > 500 || sensors[6] > 500 || sensors[7] > 500)) {
      // wall_sure = false;
      count = 0;
      led_iter = 0;
    }

    if (led_iter > 10) {
      on_led();
    }
  }
}

void show_nums(int num) {
  digitalWrite(latchPin,LOW); //ground latchPin and hold low for as long as you are transmitting
  shiftOut(dataPin,clockPin,MSBFIRST,datArray[num]);
  //return the latch pin high to signal chip that it 
  //no longer needs to listen for information
  digitalWrite(latchPin,HIGH); //pull the latchPin to save the data
}

void follow_block() {
  digitalWrite(rightMotorF, HIGH);
  digitalWrite(rightMotorB, LOW);
  analogWrite(rightMotorPWM,100);

  digitalWrite(leftMotorF, LOW);
  digitalWrite(leftMotorB, HIGH);
  analogWrite(leftMotorPWM, 100);
  digitalWrite(stby,HIGH);
  delay(200);
  while ((sensors[0] > 500 || sensors[1] > 500 || sensors[2] > 500 || sensors[3] > 500 || sensors[4] > 500 || sensors[5] > 500 || sensors[6] > 500 || sensors[7] > 500) || bounce_iter < 100) {
    qtr.readCalibrated(sensors);
    wall_follow(100);
    bounce_iter++;
  }
  bounce_iter = 0;
  num_blocks++;
  digitalWrite(rightMotorF, HIGH);
  digitalWrite(rightMotorB, LOW);
  analogWrite(rightMotorPWM, 100);
  digitalWrite(leftMotorF, HIGH);
  digitalWrite(leftMotorB, LOW);
  analogWrite(leftMotorPWM, 100);
  digitalWrite(stby,HIGH);
  delay(150);
}






// int read_line(unsigned char white_line) {
//   static int last_value;
//   unsigned char on_line = 0;
//   for(i = 0; i < num_sensors; i++) {
//     int value = sensors[i];
//     if(white_line) {
//       value = 1000 - value;
//     }

//     // keep track of whether we see the line at all
//     if(value > 200) {
//       on_line = 1;
//     }

//     // only average in values that are above a noise threshold
//     if(value > 50) {
//       avg += (long)(value) * (i * 1000);
//       sum += value;
//     }
//   }

//   if(!on_line)
//   {
//     // If it last read to the left of center, return 0.
//     if(last_value < (num_sensors - 1) * 1000 / 2)
//       return 0;

//     // If it last read to the right of center, return the max.
//     else
//       return (num_sensors - 1) * 1000;

//   }

//   last_value = avg/sum;

//   return last_value;
// }

void on_led() {
  digitalWrite(rightMotorF, HIGH);
  digitalWrite(rightMotorB, LOW);
  analogWrite(rightMotorPWM, 0);
  digitalWrite(leftMotorF, HIGH);
  digitalWrite(leftMotorB, LOW);
  analogWrite(leftMotorPWM, 0);
  digitalWrite(stby,HIGH);
  digitalWrite(led,HIGH);
  delay(5000);
  digitalWrite(led, LOW);
}

void simple_case() {
  y = analogRead(forwardSensor);
  static int j;
  // if(y < 700) {
  //   j++;
  // }
  // else {
  //   j = 0;
  //   wall_found = false;
  // }
  // if(j > 5)
  // {
  //   wall_found = true;
  // }

  if(y < 650) {
    digitalWrite(rightMotorF, HIGH);
    digitalWrite(rightMotorB, LOW);
    analogWrite(rightMotorPWM,140);

    digitalWrite(leftMotorF, LOW);
    digitalWrite(leftMotorB, HIGH);
    analogWrite(leftMotorPWM, 140);
    digitalWrite(stby,HIGH);
    delay(200);
  }
}

void wall_follow(int maximum) {
  static int lastErrorW;
  static int integralW;

  simple_case();
  if (x == 0) {
    x = 7000;
  }
  
  int errorW = int(x) - 700;
  
  integralW += errorW;
  int derivativeW = errorW - lastErrorW;
  int power_differenceW = kpW * errorW + kiW * integralW + kdW * derivativeW;
  lastErrorW = errorW;

  to_motors(power_differenceW, maximum);
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

void bias_line_follow(int maximum, bool right) {
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
  if (sensors[0] < 500 && sensors[1] < 500 && sensors[2] < 500 && sensors[3] < 500 && sensors[4] < 500 && sensors[5] < 500 && sensors[6] < 500 && sensors[7] < 500) {
    u_turn = true;
    iter_count = 0;
  }
  if (iter_count > 60) {
    u_turn = false;
  }
  iter_count++;
  if (!u_turn){
    if (right) {
      check_turn_right();
      for(int i = 0; i < 8; i++) {
        sensors[i] = 0;
      }
      sensors[0] = 1000;
    }
    else {
      check_turn_left();
      for(int i = 0; i < 8; i++) {
        sensors[i] = 0;
      }
      sensors[7] = 1000;
    }
  }
  
  to_motors(power_difference, maximum);
}

void check_turn_left() {
  if (sensors[0] > 500 && sensors[1] > 500 && sensors[2] > 500 && sensors[3] > 500 && sensors[4] > 500) {
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

void check_turn_right() {
  if (sensors[7] > 700 && sensors[6] > 700 && sensors[5] > 700 && sensors[4] > 700 && sensors[3] > 700) {
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
    analogWrite(rightMotorPWM, 0);
    digitalWrite(leftMotorF, HIGH);
    digitalWrite(leftMotorB, LOW);
    analogWrite(leftMotorPWM, 140);
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
  unsigned int position = qtr.readLine(sensors, QTR_EMITTERS_ON,0);
  for(int i=0;i<8;i++) {
    Serial.print(sensors[i]);
    Serial.print("\t");
  }
  Serial.println();
}