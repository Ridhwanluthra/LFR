void setup()

{

Serial.begin(9600);

}

void loop()

{

//waiting for input

while (Serial.available() == 0);

int val = Serial.parseInt(); //read int or parseFloat for ..float...

Serial.println(val);

}
