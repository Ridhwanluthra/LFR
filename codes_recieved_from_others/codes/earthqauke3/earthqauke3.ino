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
unsigned int value_xb;
unsigned int value_yb;
unsigned int value_zb;
unsigned int value_xupperb;
unsigned int value_yupperb;
unsigned int value_zupperb;
float j;
float xv;
float yv;
float zv;
float xvupper;
float yvupper;
float zvupper;
float xvb;
float yvb;
float zvb;
float xvupperb;
float yvupperb;
float zvupperb;
float accbase;
float accbaseupper;

void readvalues()
{
  value_x = analogRead(0);
	value_y = analogRead(1);
	value_z = analogRead(2);
       value_xupper = analogRead(3);
	value_yupper = analogRead(4);
	value_zupper= analogRead(5);

xv=(value_x/1024.0*ADC_ref-zero_x)/sensitivity_x;
yv=(value_y/1024.0*ADC_ref-zero_y)/sensitivity_y;
zv=(value_z/1024.0*ADC_ref-zero_z)/sensitivity_z;

xvupper=(value_xupper/1024.0*ADC_ref-zero_x)/sensitivity_x;
yvupper=(value_yupper/1024.0*ADC_ref-zero_y)/sensitivity_y;
zvupper=(value_zupper/1024.0*ADC_ref-zero_z)/sensitivity_z;
accbase=xvb-xv;
accbaseupper=xvupperb-xvupper;
/*Serial.print ("xvb= ");
	Serial.print (xvb);
	Serial.print(" g ");
Serial.print ("xb= ");
	Serial.print (xv);
	Serial.print(" g ");
Serial.print ("x= ");*/
if(xv>xvb)
{accbase=accbase;

  }
  else if(xv<xvb)
  {accbase=-accbase;
  }
  if(xvupper>xvupperb)
{accbaseupper=accbaseupper;

  }
  else if(xv<xvb)
  {accbaseupper=-accbaseupper;
  }
//accbase=100*sin(sqrt(pow((xvb-xv),2)+pow((yvb-yv),2)+pow((zvb-zv),2)));
	//accbaseupper=100*sin(sqrt(pow((xvupperb-xvupper),2)+pow((yvupperb-yvupper),2)+pow((zvupperb-zvupper),2)));

	Serial.print(accbase);
		Serial.print("  ");
	Serial.println(accbaseupper);
	//Serial.print("\n");

/*Serial.print ("y= ");
	Serial.print (yvb-yv);
	Serial.print(" g ");
Serial.print("\t");
Serial.print ("z= ");
	Serial.print (zvb-zv);
	Serial.print(" g ");
	Serial.print("\n");

xvupper=(value_xupper/1024.0*ADC_ref-zero_x)/sensitivity_x;
yvupper=(value_yupper/1024.0*ADC_ref-zero_y)/sensitivity_y;
zvupper=(value_zupper/1024.0*ADC_ref-zero_z)/sensitivity_z;*/   ///ooooooooooooooooooooo
/*Serial.print ("xupper= ");
	Serial.print (xvupperb-xvupper);
	Serial.print(" g ");
	Serial.print("\t");
Serial.print ("yupper= ");
	Serial.print (yvupperb-yvupper);
	Serial.print(" g ");
Serial.print("\t");
Serial.print ("zupper= ");
	Serial.print (zvupperb-zvupper);
	Serial.print(" g ");
	Serial.print("\n");
*///llllllll
//accbase=sqrt(pow((xvb-xv),2)+pow((yvb-yv),2)+pow((zvb-zv),2));
//accbaseupper=sqrt(pow((xvupperb-xvupper),2)+pow((yvupperb-yvupper),2)+pow((zvupperb-zvupper),2));

}

void setup()   {
	analogReference(INTERNAL2V56);
	Serial.begin(9600);
pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  
   value_xb = analogRead(0);
	value_yb = analogRead(1);
	value_zb = analogRead(2);
       value_xupperb = analogRead(3);
	value_yupperb = analogRead(4);
	value_zupperb= analogRead(5);
xvb=(value_xb/1024.0*ADC_ref-zero_x)/sensitivity_x;
xvupperb=(value_xupperb/1024.0*ADC_ref-zero_x)/sensitivity_x;
yvb=(value_yb/1024.0*ADC_ref-zero_y)/sensitivity_y;
yvupperb=(value_yupperb/1024.0*ADC_ref-zero_y)/sensitivity_y;
zvb=(value_zb/1024.0*ADC_ref-zero_z)/sensitivity_z;
zvupperb=(value_zupperb/1024.0*ADC_ref-zero_z)/sensitivity_z;
//accb=sqrt(pow((xvb-xv),2)+pow((yvb-yv),2)+pow((zvb-zv),2));

delay(1000);
}
void loop() {
   /*analogWrite(5, 255);  ////////////right motor at less speed so that bot moves right
    analogWrite(6,LOW);
     readvalues();
    delay(600);
 
   analogWrite(6, 255);  ////////////right motor at less speed so that bot moves right
    analogWrite(5,LOW);*/
    
     for (int i = 0; i <= 360; i=i+5) {
    
    j=255*sin(3.14*i/180);
    //Serial.println (j);
  // delay(3);
  
  if(j<0){
    analogWrite(5,-j);
    analogWrite(6,LOW);
    readvalues();
    }
    else if(j>0)
    {
    analogWrite(5,LOW);
    analogWrite(6,j);
    readvalues();
      }
    
     }
   //  delay(3);
  // put your main code here, to run repeatedly:

}
