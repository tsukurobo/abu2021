


//射出横流しArdiuno
//defense_to_ardiunoというトピックを購読


#include <ros.h>
#include "ise_motor_driver_v3.h"
#include <std_msgs/Int16.h>


int pw=0;
IseMotorDriver md(0x15);


//rosメッセージのインスタンス
ros::NodeHandle  nh;

//これがコールバック関数
void messageCb( const std_msgs::Int16 &sub_msg){
   pw=sub_msg.data;
   
}


//messageCbが受け取ったら実行する関数の名前


ros::Subscriber<std_msgs::Int16> sub("defense_to_ardiuno", &messageCb );



void setup(){
  nh.getHardware() ->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);

  IseMotorDriver::begin();
  if(!md==false) md << IseMotorDriver::createSettingData(PWM_64KHZ,SM_BRAKE_LOW_SIDE);
}

void loop(){
    nh.spinOnce();
   if (!md == false){
    md << pw;
    }
}
