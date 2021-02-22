#define ROS

//メカナムをPID制御するプログラム
#include <ros.h>
#include <abu2021_msgs/motor_pw.h>
#include <abu2021_msgs/motor_enc.h>
#include "pid_calculator.h"
#include "ise_motor_driver_v3.h"

#define MAX_PWM 255 //arduinoなら255. 今後1024とかになればいいなぁ
#define MOTOR_NUM 4 //モーターの個数

float kp[MOTOR_NUM]={1092, 1092, 1092, 1092}; //モーター4つ分のP, I, Dゲイン
float kd[MOTOR_NUM]={15, 15, 15, 15};
float ki[MOTOR_NUM]={6355, 6355, 6355, 6355};
float tdel[MOTOR_NUM]={15/1092, 15/1092, 15/1092, 15/1092};
const float enc_resolution = 1024; //エンコーダの解像度
const float wheel_radius = 0.053, pi = 3.14159265;
const float dt=0.01; //ループの周期[s]
const float a = 0.3; //エンコーダの値を微分したものを平滑するフィルタの定数

float speed_now[MOTOR_NUM] = {}, goal_vel[MOTOR_NUM]={};
int motor_pwm = 0, addrs[MOTOR_NUM] = {}; //i2cのアドレス
unsigned long time_prev;
float enc_delta[MOTOR_NUM]={}, enc_delta_pre[MOTOR_NUM]={};
long enc_now[MOTOR_NUM] = {}, enc_prev[MOTOR_NUM] = {};//1ループ前のエンコーダの値を格納する変数

PIDCalculator *PID[MOTOR_NUM];
IseMotorDriver *md[MOTOR_NUM];

#ifdef ROS
ros::NodeHandle nh;
abu2021_msgs::motor_enc enc_msg;
ros::Publisher enc_pub("encoder", &enc_msg);
////for debug////
//std_msgs::Int32 debug_data;
//ros::Publisher debug_pub("debug_info", &debug_data);
////////////////
#endif

void onReceivePower(const abu2021_msgs::motor_pw &); //速度指令が来たら呼び出される関数

void setup(){
  //ROSSerialの準備
  #ifdef ROS
  nh.getHardware()->setBaud(250000); //250kbpsで通信
  nh.initNode();
  ros::Subscriber<abu2021_msgs::motor_pw> vel_sub("motor_vel", &onReceivePower);
  nh.subscribe(vel_sub);
  nh.advertise(enc_pub);
  //nh.advertise(debug_pub);
  
  //PIDパラメータの設定
  while(!nh.connected()) { //ノードがROSのシステムに接続されるまで待つ
    nh.spinOnce();
    delay(500);
  }
  #endif
  PIDSettings pidset[4];
  #ifdef ROS
  while(!nh.getParam("/arduino_PID/kp", kp, MOTOR_NUM)){};
  while(!nh.getParam("/arduino_PID/ki", ki, MOTOR_NUM)){};
  while(!nh.getParam("/arduino_PID/kd", kd, MOTOR_NUM)){};
  while(!nh.getParam("/arduino_PID/tdel", tdel, MOTOR_NUM)){};
  #endif
  for(int i=0; i<MOTOR_NUM; i++){
    pidset[i].kp = kp[i];
    pidset[i].ki = ki[i];
    pidset[i].kd = kd[i];
    pidset[i].ts = dt;
    pidset[i].tdel = tdel[i];
    pidset[i].vmax = MAX_PWM;
    pidset[i].vmin = -MAX_PWM;
    pidset[i].mode = NORMAL;

    PID[i] = new PIDCalculator(pidset[i]);
  }

  //モードラの設定
  IseMotorDriver::begin();
  while(!nh.getParam("/motor_driver/addrs", addrs, MOTOR_NUM)){};
  for(int i=0; i<MOTOR_NUM; i++){
    md[i] = new IseMotorDriver(addrs[i]);
    *md[i] << IseMotorDriver::createSettingData(PWM_8KHZ, SM_BRAKE_LOW_SIDE);
    *md[i] >> enc_prev[i]; //初期化処理
  }

#ifdef SerialMonitor
  Serial.begin(57600);
#endif
  
  //その他
  time_prev = micros();
  //明示的に停止させる
  for(int i=0; i<MOTOR_NUM; i++) *md[i]<<0;
}

void loop(){
  // エンコーダの読み取りとPID制御の周波数は1/dt[Hz]
  unsigned long time_current = micros();
  
  if((time_current - time_prev) >= dt*1000000.0){
    for(int i=0; i<MOTOR_NUM; i++){
      //エンコーダの値を読み取り、前回のループ時との差を計算
      
      //enc_now[i] = md[i]->encoder();
      if(!(*md[i]) == false) *md[i] >> enc_now[i];
      else{
        enc_now[i]=0;
        nh.logerror("md not connected!");
      }
      enc_delta[i] = a*(float)(enc_now[i] - enc_prev[i]) + (1 - a)*enc_delta_pre[i];
      enc_prev[i] = enc_now[i];
      enc_delta_pre[i] = enc_delta[i];
      
      //PID制御実行
      speed_now[i] = -(enc_delta[i]/enc_resolution)*2*pi*wheel_radius/dt; //回転速度の計算
      motor_pwm = (int)PID[i]->calcValue(goal_vel[i] - speed_now[i]);
      //motor_pwm = 255.0*goal_vel[i]/1.33;
      *md[i] << motor_pwm; //モーターを回転させる
    }
    
    //計算した値をパブリッシュ
    #ifdef ROS
    enc_msg.e1 = speed_now[0];
    enc_msg.e2 = speed_now[1];
    enc_msg.e3 = speed_now[2];
    enc_msg.e4 = speed_now[3];
    enc_pub.publish(&enc_msg);
    #endif
    
    #ifdef SerialMonitor
    static long time_pre2 = millis();
    long time_now2 = millis();
    static uint8_t flag = 0;
    if(time_now2 - time_pre2 >= 2000){
      flag = !flag;
      for(int i=0; i<MOTOR_NUM; i++) goal_vel[i] = flag ? 0.5 : 0.2;
      time_pre2 = time_now2;
    }
    for (int i=0; i<4; i++){ Serial.print(enc_delta[i]); if(i<3) Serial.print(","); } Serial.println("");
    #endif    
    
    #ifdef ROS
    //debug_data.data = (t_now - t_pre)/1000;
    //debug_pub.publish(&debug_data);
    //t_pre = t_now;
    nh.spinOnce();
    #endif
    
    time_prev = time_current;
  }
}

void onReceivePower(const abu2021_msgs::motor_pw &v) {
  //ROSduino::getData(goal_vel, MOTOR_NUM); //目標速度の設定
  goal_vel[0] = v.v1;
  goal_vel[1] = v.v2;
  goal_vel[2] = v.v3;
  goal_vel[3] = v.v4;
  
}
