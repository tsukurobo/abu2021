//射出横流しArdiuno
//air_launch_actというトピックを購読
//air_launch_sensというトピックにエンコーダーの値をパブリッシュ


#include <ros.h>
#include "ise_motor_driver_v3.h"
//#include <abu2021_msgs/air_launch_to_ardiuno.h>
//#include <std_msgs/Int32MultiArray.h>
//#include <std_msgs/Float64.h>
#include<abu2021_msgs/launch_act.h>
#include<abu2021_msgs/launch_sens.h>

#define ben1 8//ピン番号
#define ben2 10//ピン番号
#define SOLENOID 6
#define ROCK_SENS 2

long e=0;
float degree;
int pw=0;
int sol_mode=0;
int sole=0;
int address;
int resolution;


IseMotorDriver *md;

//rosメッセージのインスタンス
ros::NodeHandle  nh;

//std_msgs::Float64 pub_msg;


abu2021_msgs::launch_sens pub_msg;
abu2021_msgs::launch_sens pub_msg_pre;

//メッセージ封筒の名前pub_msg.dataってなる

ros::Publisher chatter("launch_sens", &pub_msg);
//ros::Publisher chatter_pub=n.advertise<std_msgs::Float64>("air_launch_enc",1);



//これがコールバック関数
void messageCb( const abu2021_msgs::launch_act &sub_msg){
   pw=sub_msg.motor;
   sol_mode=sub_msg.air;
   sole=sub_msg.solenoid;
   
}



//messageCbが受け取ったら実行する関数の名前


ros::Subscriber<abu2021_msgs::launch_act> sub("launch_act", &messageCb );



void setup(){
  nh.getHardware() ->setBaud(250000);
  nh.initNode();  
  nh.advertise(chatter);
  nh.subscribe(sub);

  //#ifdef ROS
  while(!nh.getParam("/arduino/addres", &address)){};
  while(!nh.getParam("/arduino/resolution", &resolution)){};
  //#endif


  
  IseMotorDriver::begin();
  md=new IseMotorDriver((uint8_t)address);
  *md << IseMotorDriver::createSettingData(PWM_8KHZ, SM_BRAKE_LOW_SIDE);

  pinMode(ben1, OUTPUT);
  pinMode(ben2, OUTPUT);
  pinMode(SOLENOID, OUTPUT);
  pinMode(ROCK_SENS, INPUT_PULLUP);
  digitalWrite(ben1, LOW);
  digitalWrite(ben2, LOW);
  digitalWrite(SOLENOID, LOW);
}

void lowlow(){    
    digitalWrite(ben1,LOW);
    digitalWrite(ben2,LOW);
    //delay(1000);
}
void  highlow(){
    digitalWrite(ben1, HIGH);
    digitalWrite(ben2, LOW);
    //delay(1000);
}

void lowhigh(){  
    digitalWrite(ben1,LOW);
    digitalWrite(ben2,HIGH);
    //delay(1000);
}


void loop(){
   nh.spinOnce();
   if (!(*md)== false){
      *md >> e;//エンコーダーの値を取る
      degree = -e*180/resolution;   
   }else{
      degree = 0;
      nh.logerror("md  not connected!");
   }
    
  pub_msg.enc = degree;
  //str_msgデータにエンコーダーの値をいれる
  
  
  //メッセージのデータの内容をパブリッシュする
  //コールバック関数を扱う

  *md << pw;
   digitalWrite(SOLENOID, sole);
   pub_msg.touch=digitalRead(ROCK_SENS);
   
   if(pub_msg.enc != pub_msg_pre.enc || pub_msg.touch != pub_msg_pre.touch) chatter.publish( &pub_msg);
   
  //0ならばlowlow
  if     (sol_mode==0) lowlow();
  else if(sol_mode==1) highlow();
  else if(sol_mode==2) lowhigh();
    

  pub_msg_pre = pub_msg;

  delay(10);
  
}
