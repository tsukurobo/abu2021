#include <ros.h>
#include <stdlib.h>

//constant
const int open_rightSOLENOID = 2;
const int close_rightSOLENOID = 6;
const int open_leftSOLENOID = 7;
const int close_leftSOLENOID = 8;

char mode = '0';

//global variable
int delay_sol = 10; //delay time of solenoid on [milli sec]


void setup(){
  Serial.begin(9600);
  Serial.println("start");
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
  delay(100);
}

void loop(){
  //nh.spinOnce();
  if(Serial.available() > 0){
    char tmp;
    tmp = Serial.read();
    if(tmp!=10) mode = tmp;
  }
  if (mode == '1'){
    open_lefthand();
  }else if(mode == '2'){
    open_righthand();
  }else if(mode == '3'){
    close_lefthand();
  }else if(mode == '4'){
    close_righthand();
  }else if(mode == '5'){
    free_lefthand();
  }else if(mode == '6'){
    free_righthand();
  }
  mode = '0';
  delay(delay_sol);
}

//左アームを開く
void open_lefthand(){
  digitalWrite(open_leftSOLENOID, LOW);
  digitalWrite(close_leftSOLENOID, LOW);
  digitalWrite(open_leftSOLENOID, HIGH);
  Serial.println("Lefthand opened");
}

//左アームを閉じる
void close_lefthand(){
  digitalWrite(close_leftSOLENOID, LOW);
  digitalWrite(open_leftSOLENOID, LOW);
  digitalWrite(close_leftSOLENOID, HIGH);   //閉めた後開放しない
  Serial.println("Leftand closed");
}

//右アームを開く
void open_righthand(){
  //digitalWrite(open_rightSOLENOID, LOW);
  //digitalWrite(close_rightSOLENOID, LOW);
  digitalWrite(open_rightSOLENOID, HIGH);
  Serial.println("Righthand opened");
}

//右アームを閉じる
void close_righthand(){
  //digitalWrite(close_rightSOLENOID, LOW);
  //digitalWrite(open_rightSOLENOID, LOW);
  digitalWrite(close_rightSOLENOID, HIGH);   //閉めた後開放しない
  Serial.println("Righthand closed");
}

//右アームを開放
void free_righthand(){
  digitalWrite(open_rightSOLENOID, LOW);
  digitalWrite(close_rightSOLENOID, LOW);
  Serial.println("Righthand free");
}

//左アームを開放
void free_lefthand(){
  digitalWrite(open_leftSOLENOID, LOW);
  digitalWrite(close_leftSOLENOID, LOW);
  Serial.println("Lefthand free");
}
