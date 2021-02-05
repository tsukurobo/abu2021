#include "Arduino.h"
#include "encoder.h"

//リセット機能付ける！！


Encoder::Encoder(int p_a, int p_b){
  pin_a = p_a;
  pin_b = p_b;
  
  pinMode(p_a, INPUT);
  pinMode(p_b, INPUT);

  if(p_a == 18) interrupt_num_a = 5;
  else if(p_a == 19) interrupt_num_a = 4;
  else if(p_a == 20) interrupt_num_a = 3;
  else if(p_a == 21) interrupt_num_a = 2;

  if(pin_b == 18) interrupt_num_b = 5;
  else if(p_b == 19) interrupt_num_b = 4;
  else if(p_b == 20) interrupt_num_b = 3;
  else if(p_b == 21) interrupt_num_b = 2;

  //attachInterrupt(interrupt_num_a, enc_read, CHANGE);
  //attachInterrupt(interrupt_num_b, enc_read, CHANGE);
}

void Encoder::enc_read(){
  byte old = stt & B11;
  byte now = (digitalRead(pin_a) << 1) + digitalRead(pin_b);

  // change B10 & B11
  if(now == B10) now = B11;
  else if(now == B11) now = B10;
  
  stt = (old << 2) + now;

  if(old == now) return;
  else if(old == B11 && now == B00) step_cnt++;
  else if(old == B00 && now == B11) step_cnt--;
  else if(now > old) step_cnt++;
  else if(now < old) step_cnt--;
  
  Serial.print(now+8,BIN);
  Serial.print(" ");
  Serial.println(step_cnt);
}
