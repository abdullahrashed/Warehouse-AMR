#define stepPin 2
#define dirPin 3

void setup() {

 pinMode(stepPin,OUTPUT);
 pinMode(dirPin,OUTPUT);
 
}

void loop() {
 digitalWrite(dirPin,HIGH); // COUNTER CLOCKWISE 

 // 
for(int x=0 ; x<3000 ; x++)
{
  digitalWrite(stepPin,HIGH);
  delayMicroseconds(500);
  digitalWrite(stepPin,LOW);
  delayMicroseconds(500);
}
 delay(1000);
  digitalWrite(dirPin,LOW); //  CLOCKWISE 
for(int x=0 ; x<3000 ; x++)
{
  digitalWrite(stepPin,HIGH);
  delayMicroseconds(500);
  digitalWrite(stepPin,LOW);
  delayMicroseconds(500);
}
}
