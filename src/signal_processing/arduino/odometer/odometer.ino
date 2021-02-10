//#include "encoder.h"
#include <math.h>
#include "ros.h"
#include "abu2021_msgs/odom_rad.h"

#define PIN_ENC_1_A 18
#define PIN_ENC_1_B 19
#define PIN_ENC_2_A 20
#define PIN_ENC_2_B 21
#define INTERRUPT_NUM_1_A 5
#define INTERRUPT_NUM_1_B 4
#define INTERRUPT_NUM_2_A 3
#define INTERRUPT_NUM_2_B 2
#define RESOLUTION 2048.0

volatile byte stt_1_a = 0;
volatile byte stt_1_b = 0;
volatile byte stt_2_a = 0;
volatile byte stt_2_b = 0;
volatile long step_cnt_1 = 0;
volatile long step_cnt_2 = 0;

ros::NodeHandle nh;
abu2021_msgs::odom_rad odom;
ros::Publisher pub("odometer", &odom);

void setup(){
  //Serial.begin(115200);
  
  pinMode(PIN_ENC_1_A, INPUT);
  pinMode(PIN_ENC_1_B, INPUT);
  pinMode(PIN_ENC_2_A, INPUT);
  pinMode(PIN_ENC_2_B, INPUT);

  attachInterrupt(INTERRUPT_NUM_1_A, enc_read_1_a, CHANGE);
  attachInterrupt(INTERRUPT_NUM_1_B, enc_read_1_b, CHANGE);
  attachInterrupt(INTERRUPT_NUM_2_A, enc_read_2_a, CHANGE);
  attachInterrupt(INTERRUPT_NUM_2_B, enc_read_2_b, CHANGE);

  nh.initNode();
  nh.advertise(pub);
}

void loop(){
  //Serial.print(step_cnt_1*M_PI/(RESOLUTION*2));
  //Serial.print('\t');
  //Serial.println(step_cnt_2*M_PI/(RESOLUTION*2));
  nh.spinOnce();
  odom.x = step_cnt_1*M_PI/(RESOLUTION*2);
  odom.y = step_cnt_2*M_PI/(RESOLUTION*2);
  pub.publish(&odom);
}

void enc_read_1_a(){
  stt_1_a = digitalRead(PIN_ENC_1_A);
  
  if     (stt_1_a == 0 && stt_1_b == 0) step_cnt_1++;
  else if(stt_1_a == 0 && stt_1_b == 1) step_cnt_1--; 
  else if(stt_1_a == 1 && stt_1_b == 0) step_cnt_1--;
  else if(stt_1_a == 1 && stt_1_b == 1) step_cnt_1++;
}

void enc_read_1_b(){
  stt_1_b = digitalRead(PIN_ENC_1_B);
  
  if     (stt_1_a == 0 && stt_1_b == 0) step_cnt_1--;
  else if(stt_1_a == 0 && stt_1_b == 1) step_cnt_1++; 
  else if(stt_1_a == 1 && stt_1_b == 0) step_cnt_1++;
  else if(stt_1_a == 1 && stt_1_b == 1) step_cnt_1--;
}

void enc_read_2_a(){
  stt_2_a = digitalRead(PIN_ENC_2_A);
  
  if     (stt_2_a == 0 && stt_2_b == 0) step_cnt_2++;
  else if(stt_2_a == 0 && stt_2_b == 1) step_cnt_2--; 
  else if(stt_2_a == 1 && stt_2_b == 0) step_cnt_2--;
  else if(stt_2_a == 1 && stt_2_b == 1) step_cnt_2++;
}

void enc_read_2_b(){
  stt_2_b = digitalRead(PIN_ENC_2_B);
  
  if     (stt_2_a == 0 && stt_2_b == 0) step_cnt_2--;
  else if(stt_2_a == 0 && stt_2_b == 1) step_cnt_2++; 
  else if(stt_2_a == 1 && stt_2_b == 0) step_cnt_2++;
  else if(stt_2_a == 1 && stt_2_b == 1) step_cnt_2--;
}
//00
//01
//11
//10
