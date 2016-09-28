void setup() {
  // put your setup code here, to run once:
pinMode(5,OUTPUT);
Serial.begin(9600);pinMode(6,OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
digitalWrite(5,HIGH);

digitalWrite(6,LOW);
delay(1000);
digitalWrite(6,HIGH);
digitalWrite(5,LOW);
delay(1000);
}
