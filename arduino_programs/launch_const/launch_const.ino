#include <ros.h>
#include <std_msgs/Int32.h>
#include "ise_motor_driver_v3.h"
#include <math.h>

/*ピンアサインとI2Cのアドレス*/
#define SOLENOID 6
#define MD_ADDR 0x15
#define ROCK_SENS 2
#define ROCKED HIGH


/* variables
 * dist: プーリ何回転分でコンストばねを引くのを止めるか
 * enc_resol: エンコーダの分解能
 * motor_spd: 0~255
 * loop_period: 単位はミリ秒
 * state: 0なら発射台はロックされていない. 1ならロックされている
 * received_data: PCから受け取ったデータを格納
 * is_data_recieved: 0なら受け取ってない、1なら受け取っている
 * enc_pre, enc_now: エンコーダカウンタを格納
 */
const float dist = 0.5;
const uint16_t enc_resol = 4096, motor_spd = 200, loop_period = 10;
uint8_t state = 0, received_data = 0, is_data_received = 0;
long enc_pre = 0, enc_now = 0;
IseMotorDriver md(MD_ADDR);

void cmdCb(const std_msgs::Int32 &); //コールバック関数

/*ros objects*/
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int32> cmd_sub("tr_order", &cmdCb);

void setup() {
  /*Arduino側のセットアップ*/
  pinMode(SOLENOID, OUTPUT);
  pinMode(ROCK_SENS, INPUT_PULLUP); //内蔵プルアップ抵抗を有効化

  /*iseモードラのセットアップ*/
  IseMotorDriver::begin();
  md << IseMotorDriver::createSettingData(PWM_8KHZ, SM_BRAKE_LOW_SIDE);
  md << 0;
  
  /*ROS側のセットアップ*/
  nh.getHardware()->setBaud(250000); //250kbps
  nh.initNode();
  nh.subscribe(cmd_sub);
  
  /*arduinoがROSに接続されるまで待つ*/
  while(!nh.connected()){
    nh.spinOnce();
    delay(100);
  }
}

void loop() {
  nh.spinOnce();

  if(is_data_received == 1){
    is_data_received = 0;
    /*発射台がロックされていない&受信したデータが1の時、モータを回転*/
    if(state == 0 && received_data == 1){
      nh.loginfo("rotate motor");
      md << -(int)motor_spd;
      /*発射台がロックされるまでモータを回転*/
      while(1){
        /*発射台がロックされると、モータを逆回転させる。決められた分だけ回転させたらモータを停止。*/
        if(digitalRead(ROCK_SENS) == ROCKED){
          state = 1;
          delay(200);
          md << 0;
          delay(50);
          md << (int)motor_spd;

          md >> enc_pre;
          do{
            nh.spinOnce();
            /*停止指令が来たら停止させる*/
            if(is_data_received == 1 && received_data == 0){
              is_data_received = 0;
              nh.loginfo("stop");
              break;
            }
            md >> enc_now;
          }while((float)abs(enc_now - enc_pre)/enc_resol < dist);
          
          //md << 0;
          break;
        
        }else{
          //nh.loginfo("@");
          nh.spinOnce();
          /*停止指令が来たら停止させる*/
          if(is_data_received == 1 && received_data == 0){
            is_data_received = 0;
            nh.loginfo("stop");
            //md << 0;
            break;
          }
        }
        delay(loop_period);
      
      }
      md << 0;
    /*発射台がロックされていて、かつばねが引き延ばされて、かつ発射指令が受信されたとき、
    ロック解除を行う*/
    }else if(state == 1 && received_data == 2){
      nh.loginfo("launch");
      digitalWrite(SOLENOID, HIGH);
      delay(200);
      digitalWrite(SOLENOID, LOW);
      state = 0;
    }
  
  }else{
    /*発射台がロックされたらstateを1に、それ以外の時はstateを0に*/
    if(digitalRead(ROCK_SENS) == ROCKED) state = 1;
    else state = 0;
  }

  delay(loop_period);
}

void cmdCb(const std_msgs::Int32 &cmd){
  received_data = cmd.data;
  is_data_received = 1;
}
