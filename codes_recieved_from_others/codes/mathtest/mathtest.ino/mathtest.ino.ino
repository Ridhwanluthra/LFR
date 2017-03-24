int j;
float f=9.0;
long l=100;
double d=9;
void setup() {
  // put your setup code here, to run once:
pinMode(13,OUTPUT);
Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
 for (int i = 0; i <= 360; i++) {
    j=255*sin(3.14*i/180);
   // Serial.println (j);
   delay(500);
  
  if(j<0){
    analogWrite(5,-j);
    analogWrite(6,LOW);
    
    }
    else if(j>0)
    {
    analogWrite(5,LOW);
    analogWrite(6,j);
    
      }
   // analogWrite(13,j);
 }
//delay(600);
//Serial.print("");}
//Serial.println(l--);
//delay(500);

}
