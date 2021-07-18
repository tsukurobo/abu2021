#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/String.h>

#include "ise_motor_driver_v3.h"

//constant
const int open_rightSOLENOID = 5;
const int close_rightSOLENOID = 7;
const int open_leftSOLENOID = 11;
const int close_leftSOLENOID = 13;

int delay_sol = 10; //delay time of solenoid on [milli sec]

int pw_dist = 0; //妨害の速度
ros::NodeHandle nh;

std_msgs::Int16MultiArray mode;

IseMotorDriver md(0x21); //AVRのアドレスに合わせて変更

void joy_operation(const std_msgs::Int16MultiArray &mode){
  //右アームを開く
  if(mode.data[0]==2){
    open_righthand();
  }//右アームを閉じる
  else if(mode.data[0]==1){
    close_righthand();
  }//右アームをリリース
  else{
    release_righthand();
  }
  
  //左アームを開く
  if(mode.data[1]==2){
    open_lefthand();
  }//左アームを閉じる
  else if(mode.data[1]==1){
    close_lefthand();
  }//左アームをリリース
  else{
    release_lefthand();
  }
  
  pw_dist = mode.data[3];
  //妨害する
  if(mode.data[2]==1){
    disturb();
  }//妨害しない
  else{
    not_disturb();
  }
}

std_msgs::String string_msg;

ros::Subscriber<std_msgs::Int16MultiArray> sub("turn_and_dist", &joy_operation);
ros::Publisher chatter("message", &string_msg);

void setup(){
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);

  mode.data = (int*)malloc(sizeof(int)*4);
  mode.data_length = 4;

  IseMotorDriver::begin();
  if(!md==false) md << IseMotorDriver::createSettingData(PWM_64KHZ,SM_BRAKE_LOW_SIDE);
  
  //init pin
  pinMode(open_rightSOLENOID, OUTPUT);
  pinMode(close_rightSOLENOID, OUTPUT);
  pinMode(open_leftSOLENOID, OUTPUT);
  pinMode(close_leftSOLENOID, OUTPUT);

  //init actuator 全開放
  digitalWrite(open_rightSOLENOID, LOW);
  digitalWrite(close_rightSOLENOID, LOW);
  digitalWrite(open_leftSOLENOID, LOW);
  digitalWrite(close_leftSOLENOID, LOW);
}

void loop(){
  nh.spinOnce();
  chatter.publish(&string_msg);
  delay(delay_sol);
}

//左アームを開く
void open_lefthand(){
  digitalWrite(open_leftSOLENOID, LOW);
  digitalWrite(close_leftSOLENOID, LOW);
  digitalWrite(open_leftSOLENOID, HIGH);
  string_msg.data = "Lefthand opened";
}

//左アームを閉じる
void close_lefthand(){
  digitalWrite(close_leftSOLENOID, LOW);
  digitalWrite(open_leftSOLENOID, LOW);
  digitalWrite(close_leftSOLENOID, HIGH);   //閉めた後開放しない
  string_msg.data = "Lefthand closed";
}
//左アームをリリース
void release_lefthand(){
  digitalWrite(close_leftSOLENOID, LOW);
  digitalWrite(open_leftSOLENOID, LOW);
  string_msg.data = "Lefthand released";
}

//右アームを開く
void open_righthand(){
  digitalWrite(open_rightSOLENOID, LOW);
  digitalWrite(close_rightSOLENOID, LOW);
  digitalWrite(open_rightSOLENOID, HIGH);
  string_msg.data = "Righthand opened";
}

//右アームを閉じる
void close_righthand(){
  digitalWrite(close_rightSOLENOID, LOW);
  digitalWrite(open_rightSOLENOID, LOW);
  digitalWrite(close_rightSOLENOID, HIGH);   //閉めた後開放しない
  string_msg.data = "Righthand closed";
}

//左アームをリリース
void release_righthand(){
  digitalWrite(close_rightSOLENOID, LOW);
  digitalWrite(open_rightSOLENOID, LOW);
  string_msg.data = "Righthand released";
}

//妨害する
void disturb(){
  md << pw_dist;
}

//妨害しない
void not_disturb(){
  md << 0;
}
