#include "Arduino.h"
#include "encoder.h"

//リセット機能付ける！！


Encoder::Encoder(int p_a, int p_b){
  pin_a = p_a;
  pin_b = p_b;
  
  pinMode(p_a, INPUT);
  pinMode(p_b, INPUT);

  if     (p_a == 18) interrupt_num_a = 5;
  else if(p_a == 19) interrupt_num_a = 4;
  else if(p_a == 20) interrupt_num_a = 3;
  else if(p_a == 21) interrupt_num_a = 2;

  if   (pin_b == 18) interrupt_num_b = 5;
  else if(p_b == 19) interrupt_num_b = 4;
  else if(p_b == 20) interrupt_num_b = 3;
  else if(p_b == 21) interrupt_num_b = 2;

  //attachInterrupt(interrupt_num_a, Encoder::enc_read_a, CHANGE);
  //attachInterrupt(interrupt_num_b, enc_read, CHANGE);
}

void Encoder::enc_read_a(){
  stt_a = digitalRead(pin_a);
  
  if     (stt_a == 0 && stt_b == 0) step_cnt++;
  else if(stt_a == 0 && stt_b == 1) step_cnt--; 
  else if(stt_a == 1 && stt_b == 0) step_cnt--;
  else if(stt_a == 1 && stt_b == 1) step_cnt++;
}

void Encoder::enc_read_b(){
  stt_b = digitalRead(pin_b);
  
  if     (stt_a == 0 && stt_b == 0) step_cnt--;
  else if(stt_a == 0 && stt_b == 1) step_cnt++; 
  else if(stt_a == 1 && stt_b == 0) step_cnt++;
  else if(stt_a == 1 && stt_b == 1) step_cnt--;
}
