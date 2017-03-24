int i,e;
void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
}
void loop() {
  // put your main codie here, to run repeatedly:
i=analogRead(A0);
e=analogRead(A1);

Serial.println(i);

Serial.println(e);
delay(200);

}
