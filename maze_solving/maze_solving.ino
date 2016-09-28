#include <QTRSensors.h>
#include <StackArray.h>

#define rightMotorF 7
#define rightMotorB 6
#define rightMotorPWM 5
#define leftMotorF 10
#define leftMotorB 9
#define leftMotorPWM 11
#define stby 8

QTRSensorsRC qtr((unsigned char[]) {A0,4, A1, A2, A3, A4, 2, A5}, 8, 2500);

int lastError = 0;
float kp = 0.1;  // 0.08 // for small = 0.1
float kd = 1.7; // 1.0   // for small = 1.7
float ki = 0;
int integral = 0;
int derivative = 0;


/*          DEAFAULT CONFIGS
 *        
 *        * when encounter junction 3 and 6 go left first
 * 
 * 
 */



//direction :- N - 1, NE - 2, E - 3 , SE - 4, S - 5, SW - 6, W - 7, NW - 8;
int orientation;

/*      JUNCTION TYPES                                                                                  
 *       
 *   0 - no junction found    
 *       
 * 1 - ____ ____      2 -                 3 -      |                                                 
 *         |                  |                ____|                                   
 *         |              ____| ____               |                                      
 *                                                 |                             
 *                                                 
 * 4 - 
 *         |
 *      ___|        5 -                 6 -                        
 *         \                   /                 /   
 *          \             ___/               ___/            
 *                           |                  \                                 
 *                           |                   \                                    
 *                           
 *                           
 *                           
 */
int junction_type;

StackArray<int> x_coords;
StackArray<int> y_coords;
StackArray<int> next_direction;
StackArray<bool> need_for_exploration;
StackArray<int> next_explore;

StackArray<StackArray<int>> adj_matrix;


int junction_count = 0;
 
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
}

 
void loop()
{
  unsigned int sensors[8];
  
  
  int position = qtr.readLine(sensors);
    
  int error = int(position) - 3500;
  integral += error;
  derivative = error - lastError;
  int power_difference = kp * error + ki * integral + kd * derivative;
  //Serial.println(power_difference);
  lastError = error;
  
  const int maximum = 80;
  
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

void maze_solve() {
  //if junction already found do something!!!!!!!!!!!!!1
  junction_type = check_junction();
  if (junction_type != 0) {
    //check if junction already reached
    //if not then do this
    //DO THIS ONLY IF NEW JUNCTION
    junction_setting();
    need_for_exploration.push(1);
    next_explore.push(junction_count);
    junction_count++;
    update_adj_matrix();
  }
}

void update_adj_matrix() {
  // if new vertex
  StackArray<int> ex;
  ex.push(1);
  adj_matrix.push(ex);
}

void get_coords() {
  float xy[2];
  //write code for getting coords
  return xy;
}

void junction_setting() {
  //SET NEXT DIRECTION AFTER SOME TIME
  if (junction_type == 1) {
     next_direction.push(get_direction(theta + 90));
  }
  else if (junction_type == 2) {
    next_direction.push(get_direction(theta - 90));
  }
  else if (junction_type == 3) {
    next_direction.push(get_direction(theta + 180));
  }
  else if (junction_type == 4) {
    next_direction.push(get_direction(theta - 135));
  }
  else if (junction_type == 5) {
    next_direction.push(get_direction(theta + 135));
  }
  else if (junction_type == 6) {
    next_direction.push(get_direction(theta + 90));
  }
}

int get_direction() {
  if (theta > 345 && theta > 15){
    return 1;
  }
  else if (theta > 30 && theta < 60) {
    return 2;
  }
  else if (theta > 75 && theta < 105) {
    return 3;
  }
  else if (theta > 120 && theta < 150) {
    return 4;
  }
  else if (theta > 165 && theta < 195) {
    return 5;
  }
  else if (theta > 210 && theta < 240) {
    return 6;
  }
  else if (theta > 255 && theta < 285) {
    return 7;
  }
  else if (theta > 300 && theta < 330) {
    return 8;
  }
}


int check_junction() {
  private int junction_type = 0;
  //conditions to detect which junction
  if (junction_type != 0 ) {
    //FIX DELAY - USE SOMETHING ELSE;
    delay(3000);
  }
  // return junction_type
}





void stop1() {
  digitalWrite(rightMotorF,LOW);
  digitalWrite(rightMotorB, LOW);
  analogWrite(rightMotorPWM,0 );
  digitalWrite(leftMotorF, LOW);
  digitalWrite(leftMotorB, LOW);
  analogWrite(leftMotorPWM, 0);
  digitalWrite(stby,LOW);
}

void right()
  {    
   digitalWrite(rightMotorF,LOW);
  digitalWrite(rightMotorB, HIGH);
  analogWrite(rightMotorPWM,75);
    digitalWrite(leftMotorF, HIGH);
  digitalWrite(leftMotorB, LOW);
  analogWrite(leftMotorPWM, 75);
  digitalWrite(stby,HIGH);
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
