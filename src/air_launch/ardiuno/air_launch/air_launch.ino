


//射出横流しArdiuno
//air_launch_tpcというトピックを購読
//air_launch_encというトピックにエンコーダーの値をパブリッシュ
//使うメッセージはencがdoubleとカスタムメッセージair_launch_to_ardiuno.msg


#include <ros.h>
#include "ise_motor_driver_v3.h"
//#include <abu2021_msgs/air_launch_to_ardiuno.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64.h>


#define ben1 A4//ピン番号
#define ben2 A1//ピン番号

long e=0;
float degree;
int pw=0;
int sol_mode=0;

IseMotorDriver md(0x15);


//rosメッセージのインスタンス
ros::NodeHandle  nh;

std_msgs::Float64 pub_msg;

//メッセージ封筒の名前pub_msg.dataってなる

ros::Publisher chatter("air_launch_enc", &pub_msg);
//ros::Publisher chatter_pub=n.advertise<std_msgs::Float64>("air_launch_enc",1);
//air_launch_encという名前のトピックにfloat64型でメッセージ送るよ


//これがコールバック関数
void messageCb( const std_msgs::Int32MultiArray &sub_msg){
   pw=sub_msg.data[0];
   sol_mode=sub_msg.data[1];
   
}


//messageCbが受け取ったら実行する関数の名前


ros::Subscriber<std_msgs::Int32MultiArray> sub("air_launch_tpc", &messageCb );



void setup(){
  nh.getHardware() ->setBaud(115200);
  nh.initNode();  
  nh.advertise(chatter);
  nh.subscribe(sub);

  IseMotorDriver::begin();
  if(!md==false) md << IseMotorDriver::createSettingData(PWM_64KHZ,SM_BRAKE_LOW_SIDE);

  pinMode(ben1, OUTPUT);
  pinMode(ben2, OUTPUT);
  digitalWrite(ben1, LOW);
  digitalWrite(ben2, LOW);
}

void lowlow(){    
    digitalWrite(ben1,LOW);
    digitalWrite(ben2,LOW);
    delay(1000);
}
void  highlow(){
    digitalWrite(ben1, HIGH);
    digitalWrite(ben2, LOW);
    delay(1000);
}

void lowhigh(){  
    digitalWrite(ben1,LOW);
    digitalWrite(ben2,HIGH);
    delay(1000);
}


void loop(){
    nh.spinOnce();
   if (!md == false){
      md >> e;//エンコーダーの値を取る
      degree = -e*180/2048;   
    }
    
  pub_msg.data = degree;
  //str_msgデータにエンコーダーの値をいれる
  
  chatter.publish( &pub_msg);
  //メッセージのデータの内容をパブリッシュする
  //コールバック関数を扱う

  md << pw;
  //0ならばlowlow
  if(sol_mode==0) lowlow();
  if(sol_mode==1) {
    highlow();
    lowlow();
  }
  if(sol_mode==2) {
    lowhigh();
    lowlow();
    highlow();
    lowlow();
  }
  
  
}
