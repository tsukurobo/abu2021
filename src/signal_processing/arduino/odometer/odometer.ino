//#include "encoder.h"

#define PIN_ENC_A 18
#define PIN_ENC_B 19
#define INTERRUPT_NUM_A 5
#define INTERRUPT_NUM_B 4
#define RESOLUTION 48

volatile byte stt = 0;
volatile long step_cnt = 0;

void setup(){
  Serial.begin(115200);
  
  pinMode(PIN_ENC_A, INPUT);
  pinMode(PIN_ENC_B, INPUT);

  attachInterrupt(INTERRUPT_NUM_A, enc_read, CHANGE);
  attachInterrupt(INTERRUPT_NUM_B, enc_read, CHANGE);
}

void loop(){

}

void enc_read(){
  byte old = stt & B11;
  byte now = (digitalRead(PIN_ENC_A) << 1) + digitalRead(PIN_ENC_B);

  // change B10 & B11
  if(now == B10) now = B11;
  else if(now == B11) now = B10;
  
  stt = (old << 2) + now;

  //setp count update
  if(old == B11 && now == B00) step_cnt++;
  else if(old == B00 && now == B11) step_cnt--;
  else if(now > old) step_cnt++;
  else if(now < old) step_cnt--;
  
  if(abs(now-old)>1 && abs(now-old)<3) Serial.println("ERROR");

  double res = (double)step_cnt / (RESOLUTION*4);

  //Serial.println(now);
  
  Serial.print(step_cnt);
  Serial.print("\t");
  Serial.println(res);
}
