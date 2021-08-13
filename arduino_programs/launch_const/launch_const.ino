#include <ros.h>
//#include <std_msgs/Int32.h>
#include <abu2021_msgs/tr_order.h>
#include "ise_motor_driver_v3.h"
#include <math.h>

#define CONST_LAUNCH 0

/*ピンアサインとI2Cのアドレス*/
#define SOLENOID 10
#define MD_ADDR 0x15
#define ROCK_SENS 2
#define ROCKED HIGH


/* variables
 * dist: プーリ何回転分でコンストばねを引くのを止めるか
 * enc_resol: エンコーダの分解能
 * motor_spd: 0~255
 * trig_count_th: stateが1になるために、何ループROCKEDが検出されればよいか
 * loop_period: 単位はミリ秒
 * state: 0なら発射台はロックされていない. 1ならロックされている
 * received_data: PCから受け取ったデータを格納
 * is_data_recieved: 0なら受け取ってない、1なら受け取っている
 * enc_pre, enc_now: エンコーダカウンタを格納
 */
float dist = 0.3, dist2 = 1.0, dist3 = 1.5;
int trig_count = 0;
const int enc_resol = 4096, motor_spd = 220, loop_period = 10, trig_count_th = 5;
uint8_t state = 0, received_data = 0, is_data_received = 0;
long enc_pre = 0, enc_now = 0;
IseMotorDriver md(MD_ADDR);

void cmdCb(const abu2021_msgs::tr_order &); //コールバック関数

/*ros objects*/
ros::NodeHandle nh;
ros::Subscriber<abu2021_msgs::tr_order> cmd_sub("tr_order", &cmdCb);

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
  /*load params*/
  while(!nh.getParam("/const/dist",&dist, 1));
  while(!nh.getParam("/const/dist2",&dist2, 1));
  while(!nh.getParam("/const/dist3",&dist3, 1));
}

void loop() {
  nh.spinOnce();

  if(is_data_received == 1){
    is_data_received = 0;
    /*発射台がロックされていない&受信したデータが1の時、モータを回転*/
    if(state == 0 && received_data == 1){
      nh.loginfo("rotate motor");
      md << -motor_spd;
      /*発射台がロックされるまでモータを回転*/
      while(1){
        /*発射台がロックされると、モータを逆回転させる。決められた分だけ回転させたらモータを停止。*/
        if(digitalRead(ROCK_SENS) == ROCKED){
          trig_count++;
          
          if(trig_count >= trig_count_th){
            trig_count = 0;
            state = 1;
            delay(150);
            md << 0;
            delay(50);
            md << motor_spd;
  
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
            nh.spinOnce();
          }
        
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

      //ロックを解除した後、再びソレノイドを動かして発射台と連結させる
      delay(200);
      is_data_received = 1;
      received_data = 1;
    
    }else if(state == 1 && (received_data == 1 || received_data == 3 || received_data == 4)){
      float goal_d = 0, d_now = 0, delta = 0, delta_pre = 0; 
      if(received_data == 1) goal_d = dist;
      else if(received_data == 3) goal_d = dist2;
      else if(received_data == 4) goal_d = dist3;

      md >> enc_now;
      d_now = (float)abs(enc_now-enc_pre)/enc_resol;
      
      if(fabs(d_now-goal_d)>0.01){
        if(d_now > goal_d) md << -motor_spd;
        else if(d_now < goal_d) md << motor_spd;
        
        while(1){
          md >> enc_now;
          d_now = (float)abs(enc_now-enc_pre)/enc_resol;
          delta = d_now - goal_d;
          if(delta*delta_pre<0) break;
          delta_pre = delta;
          
          nh.spinOnce();
          if(is_data_received == 1 && received_data == 0){
            is_data_received = 0;
            break;
          }
          
          delay(loop_period);
        }
        
        md << 0;
      }
    }
  
  }else{
    /*発射台がロックされたらstateを1に、それ以外の時はstateを0に*/
    if(digitalRead(ROCK_SENS) == ROCKED) state = 1;
    else state = 0;
  }

  delay(loop_period);
}

void cmdCb(const abu2021_msgs::tr_order &cmd){
  if(cmd.nodeId == CONST_LAUNCH){
    received_data = cmd.orderId;
    is_data_received = 1;
  }
}
