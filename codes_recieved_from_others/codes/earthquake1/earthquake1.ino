int x,y,z;
void setup() {
  pinMode(3, OUTPUT);
  pinMode(6, OUTPUT);
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  
  analogWrite(3, 255);  ////////////right motor at less speed so that bot moves right
    analogWrite(6,LOW);
    delay(600);
    analogWrite(6, 255);
    analogWrite(13,LOW);
    delay(600);
    digitalWrite(13,LOW);
    delay(1000);
    digitalWrite(13,HIGH);
   
  // put your main code here, to run repeatedly:/*
/*x=analogRead(A0);
y=analogRead(A1);
z=analogRead(A2);
Serial.println("x");
Serial.println(x);
Serial.println("y");
Serial.println(y);
Serial.println("z");
Serial.println(z);
//delay(500);
*/
}
