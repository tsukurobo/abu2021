

//topicをサブスクライブ、pwもらう,[1]が１なら発射する
//pub_encにエンコーダーの値をパブリッシュ
//横流し

#include <ros.h>
//#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include "ise_motor_driver_v3.h"
#define benn  A4//ピン番号
#define benn2 A1//ピン番号


//メッセージファイルのインクルード
long e=0;
float kakudo;
int pw=0;
int mode_sol=0;

IseMotorDriver md(0x21);


//rosメッセージのインスタンス
ros::NodeHandle  nh;
//chatterという名前のトピック名のパブリッシャのインスタンスを作る
//ROSメッセージの型

std_msgs::Int32 pub_msg;
//メッセージの中身の変数名を決める
ros::Publisher chatter("pub_enc", &pub_msg);



void messageCb( const std_msgs::Int32MultiArray &sub_msg) {
   pw=sub_msg.data[0];

   if(sub_msg.data[1]==1){
    mode_sol=1;    
   }
   
   if(sub_msg.data[1]==2){
    mode_sol=2;
   }
}


//messageCbが受け取ったら実行する関数の名前


ros::Subscriber<std_msgs::Int32MultiArray> sub("topic", &messageCb );



void setup()
{
  nh.getHardware() ->setBaud(115200);
  nh.initNode();  
  nh.advertise(chatter);
  nh.subscribe(sub);

  IseMotorDriver::begin();
  if(!md==false) md << IseMotorDriver::createSettingData(PWM_64KHZ,SM_BRAKE_LOW_SIDE);

  pinMode(benn, OUTPUT);
  pinMode(benn2, OUTPUT);
  digitalWrite(benn, LOW);
  digitalWrite(benn2, LOW);
}

void loop(){
    nh.spinOnce();
   if (!md == false){
      md >> e;//エンコーダーの値を取る
      kakudo=-e*180/2048;      
    }
    
  pub_msg.data = e;
  //str_msgデータにエンコーダーの値をいれる
  
  chatter.publish( &pub_msg);
  //メッセージのデータの内容をパブリッシュする
  //コールバック関数を扱う

  md << pw;
  delay(30);
  if(mode_sol==1){
    delay(500);
    digitalWrite(benn, HIGH); //
    digitalWrite(benn2, LOW);
    delay(100);
    digitalWrite(benn,LOW);
    digitalWrite(benn2,LOW);
    delay(1000);
    mode_sol=0;
  }
  
  if(mode_sol==2){
    delay(500);
    digitalWrite(benn,LOW);
    digitalWrite(benn2,HIGH);
    delay(100);
    digitalWrite(benn,LOW);
    digitalWrite(benn2,LOW);
    delay(1000);
    mode_sol=0;
  }
}
