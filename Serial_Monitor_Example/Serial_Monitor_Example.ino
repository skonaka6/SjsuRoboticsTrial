// following this tutorial
// https://www.circuitbasics.com/how-to-read-user-input-from-the-arduino-serial-monitor/

int num;
int i=0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print('.');
  if (Serial.available()){
    num = Serial.parseInt();
    Serial.println();
    Serial.print("num is now: ");
    Serial.println(num);
    i = 0;
  }
  else
  {
    i++;
    if (i>100){
      i = 0;
      Serial.println();
    }
  }
  delay(500);
}
