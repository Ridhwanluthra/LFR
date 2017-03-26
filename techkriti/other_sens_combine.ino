#include <NewPing.h>

#define rightMotorF 7
#define rightMotorB 8
#define rightMotorPWM 9
#define leftMotorF 4
#define leftMotorB 5
#define leftMotorPWM 3
// #define rightSensor A7
#define forwardSensor A6
#define stby 6
#define led 13

#define s1 A0
#define s2 A1
#define s3 A2
#define s4 A3
#define s5 A4

#define num_sensors 5

#define TRIGGER_PIN  10
#define ECHO_PIN     2
#define MAX_DISTANCE 200

const int latchPin = 1;//Pin 12 connected to ST_CP of 74HC595
const int clockPin = 2;//Pin 8 connected to SH_CP of 74HC595 
const int dataPin = 0; //Pin 11 connected to DS of 74HC595 
//display 0,1,2,3,4,5,6,7,8,9,A,b,C,d,E,F
int datArray[16] = {252, 96, 218, 242, 102, 182, 190, 224, 254, 246, 238, 62, 156, 122, 158, 142};

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

void setup() {
  pinMode(rightMotorF, OUTPUT);
  pinMode(rightMotorB, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotorF, OUTPUT);
  pinMode(leftMotorB, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  // pinMode(rightSensor, INPUT);
  pinMode(forwardSensor, INPUT);
  pinMode(led,OUTPUT);
  pinMode(stby,OUTPUT);

  pinMode(s1, INPUT);
  pinMode(s2, INPUT);
  pinMode(s3, INPUT);
  pinMode(s4, INPUT);
  pinMode(s5, INPUT);

  pinMode(latchPin,OUTPUT);
  pinMode(clockPin,OUTPUT);
  pinMode(dataPin,OUTPUT);
  
  // Serial.begin(9600);
}

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
int bounce_iter = 0;
int num_blocks = 0;

int ll, l, m , r, rr;
void loop() {
  x = sonar.ping();
  y = analogRead(forwardSensor);

  ll = digitalRead(s1);
  l = digitalRead(s2);
  r = digitalRead(s3);
  m = digitalRead(s4);
  rr = digitalRead(s5);
  


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
    if (sensors[0] > 500 && sensors[1] > 500 && sensors[2] > 500 && sensors[3] > 500 && sensors[4] > 500 && sensors[5] > 500 && sensors[6] > 500 && sensors[7] > 500) {
      led_iter++;
    }
    if (led_iter > 10) {
      on_led();
    }
    if (x > set_distance && (sensors[0] > 500 || sensors[1] > 500 || sensors[2] > 500 || sensors[3] > 500 || sensors[4] > 500 || sensors[5] > 500 || sensors[6] > 500 || sensors[7] > 500)) {
      wall_sure = false;
      count = 0;
      led_iter = 0;
    }
  }











  // if (rr == 0) {
  //   digitalWrite(rightMotorF, HIGH);
  //   digitalWrite(rightMotorB, LOW);
  //   analogWrite(rightMotorPWM, 0);
  //   digitalWrite(leftMotorF, HIGH);
  //   digitalWrite(leftMotorB, LOW);
  //   analogWrite(leftMotorPWM, 0);
  //   digitalWrite(stby,HIGH);
  //   delay(30);

  //   digitalWrite(rightMotorF, HIGH);
  //   digitalWrite(rightMotorB, LOW);
  //   analogWrite(rightMotorPWM, 0);
  //   digitalWrite(leftMotorF, HIGH);
  //   digitalWrite(leftMotorB, LOW);
  //   analogWrite(leftMotorPWM, 100);
  //   digitalWrite(stby,HIGH);
  //   delay(300);

  //   while(m != 0) {
  //     m = digitalRead(s4);
  //     digitalWrite(rightMotorF, HIGH);
  //     digitalWrite(rightMotorB, LOW);
  //     analogWrite(rightMotorPWM, 0);
  //     digitalWrite(leftMotorF, HIGH);
  //     digitalWrite(leftMotorB, LOW);
  //     analogWrite(leftMotorPWM, 100);
  //     digitalWrite(stby,HIGH);
  //   }
  // }

  // if (ll == 0) {
  //   digitalWrite(rightMotorF, HIGH);
  //   digitalWrite(rightMotorB, LOW);
  //   analogWrite(rightMotorPWM, 0);
  //   digitalWrite(leftMotorF, HIGH);
  //   digitalWrite(leftMotorB, LOW);
  //   analogWrite(leftMotorPWM, 0);
  //   digitalWrite(stby,HIGH);
  //   delay(30);

  //   digitalWrite(rightMotorF, HIGH);
  //   digitalWrite(rightMotorB, LOW);
  //   analogWrite(rightMotorPWM, 100);
  //   digitalWrite(leftMotorF, HIGH);
  //   digitalWrite(leftMotorB, LOW);
  //   analogWrite(leftMotorPWM, 0);
  //   digitalWrite(stby,HIGH);
  //   delay(300);

  //   while(m != 0) {
  //     m = digitalRead(s4);
  //     digitalWrite(rightMotorF, HIGH);
  //     digitalWrite(rightMotorB, LOW);
  //     analogWrite(rightMotorPWM, 100);
  //     digitalWrite(leftMotorF, HIGH);
  //     digitalWrite(leftMotorB, LOW);
  //     analogWrite(leftMotorPWM, 0);
  //     digitalWrite(stby,HIGH);
  //   }
  // }




  if (rr == 0) {
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
    analogWrite(leftMotorPWM, 100);
    digitalWrite(stby,HIGH);
    delay(300);

    catch_right();
  }

  if (ll == 0) {
    digitalWrite(rightMotorF, HIGH);
    digitalWrite(rightMotorB, LOW);
    analogWrite(rightMotorPWM, 0);
    digitalWrite(leftMotorF, HIGH);
    digitalWrite(leftMotorB, LOW);
    analogWrite(leftMotorPWM, 0);
    digitalWrite(stby,HIGH);
    delay(30);

    ll = digitalRead(s1);
    l = digitalRead(s2);
    r = digitalRead(s3);
    m = digitalRead(s4);
    rr = digitalRead(s5);
    if (l == 1 && m == 1 && r == 1) {
      digitalWrite(rightMotorF, HIGH);
      digitalWrite(rightMotorB, LOW);
      analogWrite(rightMotorPWM, 100);
      digitalWrite(leftMotorF, HIGH);
      digitalWrite(leftMotorB, LOW);
      analogWrite(leftMotorPWM, 0);
      digitalWrite(stby,HIGH);
      delay(300);

      catch_left();
    }
  }





  // found block
  if(y < 800) {
    follow_block();
  }

  line_follow();
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
  while ((ll == 1 && l == 1 && m == 1 && r == 1 && rr == 1) || bounce_iter < 100) {
    ll = digitalRead(s1);
    l = digitalRead(s2);
    r = digitalRead(s3);
    m = digitalRead(s4);
    rr = digitalRead(s5);
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

  // digitalWrite(rightMotorF, HIGH);
  // digitalWrite(rightMotorB, LOW);
  // analogWrite(rightMotorPWM,0);

  // digitalWrite(leftMotorF, LOW);
  // digitalWrite(leftMotorB, HIGH);
  // analogWrite(leftMotorPWM, 0);
  // digitalWrite(stby,HIGH);
  // delay(1000);
  catch_left();
}

void show_nums(int num) {
  digitalWrite(latchPin,LOW); //ground latchPin and hold low for as long as you are transmitting
  shiftOut(dataPin,clockPin,MSBFIRST,datArray[num]);
  //return the latch pin high to signal chip that it 
  //no longer needs to listen for information
  digitalWrite(latchPin,HIGH); //pull the latchPin to save the data
}

void line_follow() {
  if (l == 0 && m == 0 && r == 0) {
    sstop();
  }
  else if (l == 0 && m == 0 && r == 1) {
    left();
  }
  else if (l == 0 && m == 1 && r == 0) {
    sstop();
  }
  else if (l == 0 && m == 1 && r == 1) {
    soft_left();
  }
  else if (l == 1 && m == 0 && r == 0) {
    right();
  }
  else if (l == 1 && m == 0 && r == 1) {
    forward();
  }
  else if (l == 1 && m == 1 && r == 0) {
    soft_right();
  }
  else if (l == 1 && m == 1 && r == 1) {
    forward();
  }
}

void forward() {
  digitalWrite(rightMotorF, HIGH);
  digitalWrite(rightMotorB, LOW);
  analogWrite(rightMotorPWM, 100);
  digitalWrite(leftMotorF, HIGH);
  digitalWrite(leftMotorB, LOW);
  analogWrite(leftMotorPWM, 100);
  digitalWrite(stby,HIGH);
}

void left() {
  digitalWrite(rightMotorF, HIGH);
  digitalWrite(rightMotorB, LOW);
  analogWrite(rightMotorPWM, 100);
  digitalWrite(leftMotorF, HIGH);
  digitalWrite(leftMotorB, LOW);
  analogWrite(leftMotorPWM, 0);
  digitalWrite(stby,HIGH);
}

void soft_left() {
  digitalWrite(rightMotorF, HIGH);
  digitalWrite(rightMotorB, LOW);
  analogWrite(rightMotorPWM, 100);
  digitalWrite(leftMotorF, HIGH);
  digitalWrite(leftMotorB, LOW);
  analogWrite(leftMotorPWM, 50);
  digitalWrite(stby,HIGH);
}

void right() {
  digitalWrite(rightMotorF, HIGH);
  digitalWrite(rightMotorB, LOW);
  analogWrite(rightMotorPWM, 0);
  digitalWrite(leftMotorF, HIGH);
  digitalWrite(leftMotorB, LOW);
  analogWrite(leftMotorPWM, 100);
  digitalWrite(stby,HIGH);
}

void soft_right() {
  digitalWrite(rightMotorF, HIGH);
  digitalWrite(rightMotorB, LOW);
  analogWrite(rightMotorPWM, 50);
  digitalWrite(leftMotorF, HIGH);
  digitalWrite(leftMotorB, LOW);
  analogWrite(leftMotorPWM, 100);
  digitalWrite(stby,HIGH);
}

void sstop() {
  digitalWrite(rightMotorF, HIGH);
  digitalWrite(rightMotorB, LOW);
  analogWrite(rightMotorPWM, 0);
  digitalWrite(leftMotorF, HIGH);
  digitalWrite(leftMotorB, LOW);
  analogWrite(leftMotorPWM, 0);
  digitalWrite(stby,HIGH);
}

void catch_left() {
  while(m != 0) {
    m = digitalRead(s4);
    digitalWrite(rightMotorF, HIGH);
    digitalWrite(rightMotorB, LOW);
    analogWrite(rightMotorPWM, 100);
    digitalWrite(leftMotorF, HIGH);
    digitalWrite(leftMotorB, LOW);
    analogWrite(leftMotorPWM, 0);
    digitalWrite(stby,HIGH);
  }
}

void catch_right() {
  while(m != 0) {
    m = digitalRead(s4);
    digitalWrite(rightMotorF, HIGH);
    digitalWrite(rightMotorB, LOW);
    analogWrite(rightMotorPWM, 100);
    digitalWrite(leftMotorF, HIGH);
    digitalWrite(leftMotorB, LOW);
    analogWrite(leftMotorPWM, 0);
    digitalWrite(stby,HIGH);
  }
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
    analogWrite(rightMotorPWM,100);

    digitalWrite(leftMotorF, LOW);
    digitalWrite(leftMotorB, HIGH);
    analogWrite(leftMotorPWM, 100);
    digitalWrite(stby,HIGH);
    delay(200);
  }
}

void wall_follow(int maximum) {
  x = sonar.ping();
  y = analogRead(forwardSensor);

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
