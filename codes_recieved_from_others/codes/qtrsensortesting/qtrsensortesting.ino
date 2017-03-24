#include <QTRSensors.h>
#define NUM_SENSORS 8    // number of sensors used
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low

// Create instance of sensors. Sensors connected to pins 5 to 12
//QTRSensorsRC qtrrc((unsigned char[]) {A2,A3,A4,A5},  NUM_SENSORS, TIMEOUT); //QTR_NO_EMITTER_PIN); 
QTRSensorsRC qtrrc((unsigned char[]) {A0,A1,A2,A3,A4,A5,10,11},  NUM_SENSORS, TIMEOUT); //QTR_NO_EMITTER_PIN); 

unsigned int sensorValues[NUM_SENSORS];
void setup() {
  // put your setup code here, to run once:
pinMode(10, INPUT);
  pinMode(11, INPUT);
  Serial.begin(9600);
  for (int k = 0; k < 150; k++){ // make the calibration take about 5 seconds
qtrrc.calibrate();
Serial.println(k);
delay(20);}
}

void loop() {
  // put your main code here, to run repeatedly:
qtrrc.readCalibrated(sensorValues);
  
  // print the sensor values as numbers from 0 to 1023, where 0 means maximum reflectance and
  // 1023 means minimum reflectance
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
  }
  Serial.println();
  
  delay(250);
}
