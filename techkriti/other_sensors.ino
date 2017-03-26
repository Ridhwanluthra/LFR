#include <NewPing.h>

#define rightMotorF 7
#define rightMotorB 8
#define rightMotorPWM 9
#define leftMotorF 4
#define leftMotorB 5
#define leftMotorPWM 3
#define rightSensor A7
#define forwardSensor A5
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

  pinMode(s1, INPUT);
  pinMode(s2, INPUT);
  pinMode(s3, INPUT);
  pinMode(s4, INPUT);
  pinMode(s5, INPUT);
  
  // Serial.begin(9600);
}

//=====Constants for Line Following====================================================
#define kp 50
#define kd 0
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

#define set_distance 1200

void loop() {
  line_follow(70);
  // x = sonar.ping();
  // y = analogRead(forwardSensor);

  // if(x < set_distance) {
  //   count++;
  // }
  // else {
  //   count = 0;
  // }
  // if(count > 10)
  // {
  //   wall_sure = true;
  // }
  // if (y < 750) {
  //   wall_sure = true;
  //   simple_case();
  // }

  

  // if (!wall_sure) {
  //   bias_line_follow(100, true);
  // }
  // else {
  //   wall_follow(100);
  //   unsigned int position = qtr.readLine(sensors, QTR_EMITTERS_ON,0);
  //   if (sensors[0] > 500 && sensors[1] > 500 && sensors[2] > 500 && sensors[3] > 500 && sensors[4] > 500 && sensors[5] > 500 && sensors[6] > 500 && sensors[7] > 500) {
  //     led_iter++;
  //   }
  //   if (led_iter > 10) {
  //     on_led();
  //   }
  //   if (x > set_distance && (sensors[0] > 500 || sensors[1] > 500 || sensors[2] > 500 || sensors[3] > 500 || sensors[4] > 500 || sensors[5] > 500 || sensors[6] > 500 || sensors[7] > 500)) {
  //     wall_sure = false;
  //     count = 0;
  //     led_iter = 0;
  //   }
  // }
}

int read_line() {
  int ll, l, m, r, rr;
  long avg, sum;
  static int last_value;
  ll = digitalRead(s1);
  l = digitalRead(s2);
  m = digitalRead(s3);
  r = digitalRead(s4);
  rr = digitalRead(s5);

  unsigned char on_line = 0;
  
  // if(white_line) {
  //   ll = 1 - ll;
  //   l = 1 - l;
  //   m = 1 - m;
  //   r = 1 - r;
  //   rr = 1 - rr;
  // }

  // keep track of whether we see the line at all
  if(ll == 1 || l == 1 || m == 1 || r == 1 || rr == 1) {
    on_line = 1;
  }

  avg = (long)l * 1 + (long)m * 2 + (long)r * 3 + (long)rr * 4;
  sum = (int)ll + (int)l + (int)m + (int)r + (int)rr;

  if(!on_line)
  {
    // If it last read to the left of center, return 0.
    if(last_value < 2)
      return 0;

    // If it last read to the right of center, return the max.
    else
      return 4;

  }

  last_value = avg/sum;

  return last_value;
}

void line_follow(int maximum) {
  static int lastError;
  static int integral;
  int position = read_line();

  int error = position - 2;
  
  integral += error;
  int derivative = error - lastError;
  int power_difference = kp * error + ki * integral + kd * derivative;
  lastError = error;
  
  to_motors(-power_difference, maximum);
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