#define encoder_pin 2
volatile long ticks=0;
unsigned long prev=0;
float speed_=0;
void setup() {
 pinMode(encoder_pin,INPUT_PULLUP);
 attachInterrupt(digitalPinToInterrupt(encoder_pin),count_ticks,RISING);
 Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long current=millis();
  if(current-prev>=1000)
  {
    speed_=(float)ticks/(float)(current-prev)*1000.0;
    ticks=0;
    prev=current;
    Serial.println(speed_);
  }

}
void count_ticks()
{
  ticks++;
}
