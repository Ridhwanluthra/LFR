#define ADC_ref 2.56
#define zero_x 1.569
#define zero_y 1.569
#define zero_z 1.569
#define sensitivity_x 0.3
#define sensitivity_y 0.3
#define sensitivity_z 0.3
unsigned int value_x;
unsigned int value_y;
unsigned int value_z;
unsigned int value_xupper;
unsigned int value_yupper;
unsigned int value_zupper;

float xv;
float yv;
float zv;
float xvupper;
float yvupper;
float zvupper;
float angle_x;
float angle_y;
float angle_z;
void setup()   {
	analogReference(INTERNAL2V56);
	Serial.begin(9600);
pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
}
	 
	void loop() {
  
  analogWrite(5, 255);  ////////////right motor at less speed so that bot moves right
    analogWrite(6,LOW);
    delay(600);
    	
       value_x = analogRead(0);
	value_y = analogRead(1);
	value_z = analogRead(2);
       value_xupper = analogRead(3);
	value_yupper = analogRead(4);
	value_zupper = analogRead(5);
	xv=(value_x/1024.0*ADC_ref-zero_x)/sensitivity_x;
        Serial.print ("x= ");
	Serial.print (xv);
	Serial.print(" g ");
Serial.print("      ");

     xvupper=(value_xupper/1024.0*ADC_ref-zero_x)/sensitivity_x;

       Serial.print ("xupper= ");
	Serial.print (xvupper);
	Serial.print(" g ");
Serial.print("\n");

	yv=(value_y/1024.0*ADC_ref-zero_y)/sensitivity_y;
	Serial.print ("y= ");
	Serial.print (yv);
	Serial.print(" g ");
Serial.print("      ");
//Serial.print("\n");

yvupper=(value_yupper/1024.0*ADC_ref-zero_y)/sensitivity_y;
	Serial.print ("yupper= ");
	Serial.print (yvupper);
	Serial.print(" g ");
	Serial.print("\n");
	

zv=(value_z/1024.0*ADC_ref-zero_z)/sensitivity_z;
	Serial.print ("z= ");
	Serial.print (zv);
	Serial.print(" g ");
Serial.print("      ");
	//Serial.print("\n");
//Serial.print("\n");

zvupper=(value_zupper/1024.0*ADC_ref-zero_z)/sensitivity_z;
	Serial.print ("zupper= ");
	Serial.print (zvupper);
	Serial.print(" g ");
	Serial.print("\n");
analogWrite(6, 255);
    analogWrite(5,LOW);
    delay(600);
    Serial.print ("x= ");
	Serial.print (xv);
	Serial.print(" g ");
xvupper=(value_xupper/1024.0*ADC_ref-zero_x)/sensitivity_x;
Serial.print("      ");
       Serial.print ("xupper= ");
	Serial.print (xvupper);
	Serial.print(" g ");
		Serial.print("\n");

yv=(value_y/1024.0*ADC_ref-zero_y)/sensitivity_y;
	Serial.print ("y= ");
	Serial.print (yv);
	Serial.print(" g ");
//Serial.print("\n");
Serial.print("      ");
yvupper=(value_yupper/1024.0*ADC_ref-zero_y)/sensitivity_y;
	Serial.print ("yupper= ");
	Serial.print (yvupper);
	Serial.print(" g ");
Serial.print("\n");

	zv=(value_z/1024.0*ADC_ref-zero_z)/sensitivity_z;
	Serial.print ("z= ");
	Serial.print (zv);
	Serial.print(" g ");
Serial.print("      ");
	//erial.print("\n");Serial.print("      ");
//Serial.print("\n");

zvupper=(value_zupper/1024.0*ADC_ref-zero_z)/sensitivity_z;
	Serial.print ("zupper= ");
	Serial.print (zvupper);
	Serial.print(" g ");
	Serial.print("\n");

/*	Serial.print("Rotation ");
	Serial.print("x= ");
	angle_x =atan2(-yv,-zv)*57.2957795+180;
	Serial.print(angle_x);
	Serial.print(" deg");
	Serial.print(" ");
	Serial.print("y= ");
	angle_y =atan2(-xv,-zv)*57.2957795+180;
	Serial.print(angle_y);
	Serial.print(" deg");
	Serial.print(" ");
	Serial.print("z= ");
	angle_z =atan2(-yv,-xv)*57.2957795+180;
	Serial.print(angle_z);
	Serial.print(" deg");
	Serial.print("\n");*/
	}
