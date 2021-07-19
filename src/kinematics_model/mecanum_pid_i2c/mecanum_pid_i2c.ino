#define ROS

//メカナムをPID制御するプログラム
#include <ros.h>
#include <abu2021_msgs/motor_pw.h>
//#include <abu2021_msgs/cmd_vw.h>
#include <abu2021_msgs/motor_enc.h>
#include "pid_calculator.h"
#include "ise_motor_driver_v3.h"

//#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>

#define MAX_PWM 255 //arduinoなら255. 今後1024とかになればいいなぁ
#define MOTOR_NUM 4 //モーターの個数
#define SQRT2 1.41421356

float kp[MOTOR_NUM]={1092, 1092, 1092, 1092}; //モーター4つ分のP, I, Dゲイン
float kd[MOTOR_NUM]={15, 15, 15, 15};
float ki[MOTOR_NUM]={6355, 6355, 6355, 6355};
float tdel[MOTOR_NUM]={15/1092, 15/1092, 15/1092, 15/1092};
float dtoc, width, height, acc, wheel_radius = 0.053;
//int to_zero_th = 0; //0とするPWMのdutyの範囲(片側)
const float enc_resolution = 1024; //i5oエンコーダの解像度
const float pi = 3.14159265;
const float dt=0.01; //ループの周期[s]
const float a = 0.3; //エンコーダの値を微分したものを平滑するフィルタの定数

float speed_now[MOTOR_NUM] = {}, goal_vel[MOTOR_NUM]={}, v_raw[MOTOR_NUM] = {}, v_pre[MOTOR_NUM] = {};
float enc_delta[MOTOR_NUM]={}, enc_delta_pre[MOTOR_NUM]={};
float vx = 0, vy = 0, vth = 0, vx_pre = 0, vy_pre = 0, vth_pre = 0, vx_raw = 0, vy_raw = 0, vth_raw = 0;
int mode, motor_pwm = 0, addrs[MOTOR_NUM] = {}; //i2cのアドレス
unsigned long time_prev;
long enc_now[MOTOR_NUM] = {}, enc_prev[MOTOR_NUM] = {};//1ループ前のエンコーダの値を格納する変数

PIDCalculator *PID[MOTOR_NUM];
IseMotorDriver *md[MOTOR_NUM];

#ifdef ROS
ros::NodeHandle nh;
abu2021_msgs::motor_enc enc_msg;
ros::Publisher enc_pub("encoder", &enc_msg);
////for debug////
//std_msgs::Float32 debug_data;
//ros::Publisher debug_pub("debug_info", &debug_data);
//geometry_msgs::Vector3 pid_msg;
//ros::Publisher pid_pub("pid_info", &pid_msg);
////////////////
#endif

void onReceivePower(const abu2021_msgs::motor_pw &); //速度指令が来たら呼び出される関数
//float step_func(float);

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
  //while(!nh.getParam("/model/to_zero_th", &to_zero_th, 1)){};
  //while(!nh.getParam("/model/mode", &mode, 1)){};
  //while(!nh.getParam("/model/acc", &acc, 1)){};
  while(!nh.getParam("/base/model/wheel_radius", &wheel_radius, 1)){};
  /*if(mode == 1){ //TRモード時
    while(!nh.getParam("/model/width", &width, 1)){};
    while(!nh.getParam("/model/height", &height, 1)){}; 
  }else if(mode == 0){ //DRモード時
    while(!nh.getParam("/model/distance_to_center", &dtoc, 1)){};
  }*/
  while(!nh.getParam("/base/velPID/kp", kp, MOTOR_NUM)){};
  while(!nh.getParam("/base/velPID/ki", ki, MOTOR_NUM)){};
  while(!nh.getParam("/base/velPID/kd", kd, MOTOR_NUM)){};
  while(!nh.getParam("/base/velPID/tdel", tdel, MOTOR_NUM)){};
  #endif
  for(int i=0; i<MOTOR_NUM; i++){
    pidset[i].kp = kp[i];
    pidset[i].ki = ki[i];
    pidset[i].kd = kd[i];
    pidset[i].ts = dt;
    pidset[i].a1 = 3.1;
    pidset[i].a2 = 27.1;
    pidset[i].tdel = tdel[i];
    pidset[i].vmax = MAX_PWM;
    pidset[i].vmin = -MAX_PWM;
    pidset[i].mode = NORMAL;

    PID[i] = new PIDCalculator(pidset[i]);
  }

  //モードラの設定
  IseMotorDriver::begin();
  while(!nh.getParam("/base/motor_driver/addrs", addrs, MOTOR_NUM)){};
  for(int i=0; i<MOTOR_NUM; i++){
    md[i] = new IseMotorDriver(addrs[i]);
    *md[i] << IseMotorDriver::createSettingData(PWM_8KHZ, SM_BRAKE_LOW_SIDE);
    *md[i] >> enc_prev[i]; //初期化処理
  }

  //その他
  time_prev = micros();
  //明示的に停止させる
  for(int i=0; i<MOTOR_NUM; i++) *md[i]<<0;
}

void loop(){
  // エンコーダの読み取りとPID制御の周波数は1/dt[Hz]
  unsigned long time_current = micros();
  
  if((time_current - time_prev) >= dt*1000000.0 && nh.connected()){
    /*//台形制御
    if(vx < vx_raw) vx = vx_pre + acc*dt;
    else if(vx > vx_raw) vx = vx_pre - acc*dt;
    if((vx - vx_raw)*(vx_pre - vx_raw) <= 0) vx = vx_raw;

    if(vy < vy_raw) vy = vy_pre + acc*dt;
    else if(vy > vy_raw) vy = vy_pre - acc*dt;
    if((vy - vy_raw)*(vy_pre - vy_raw) <= 0) vy = vy_raw;

    if(vth < vth_raw) vth = vth_pre + 2.0*acc*dt;
    else if(vth > vth_raw) vth = vth_pre - 2.0*acc*dt;
    if((vth - vth_raw)*(vth_pre - vth_raw) <= 0) vth = vth_raw;
    vx_pre = vx;
    vy_pre = vy;
    vth_pre = vth;

    if(mode == 0){
        //4輪オムニ
        goal_vel[0] = 0.5*SQRT2*vy + 0.5*SQRT2*vx + dtoc*vth;
        goal_vel[1] = 0.5*SQRT2*vy - 0.5*SQRT2*vx + dtoc*vth;
        goal_vel[2] = -0.5*SQRT2*vy - 0.5*SQRT2*vx + dtoc*vth;
        goal_vel[3] = -0.5*SQRT2*vy + 0.5*SQRT2*vx + dtoc*vth;
    }
    else if(mode == 1){
    //メカナム
        goal_vel[0] = vy + vx + 0.5*(vth)*(width+height);
        goal_vel[1] = vy - vx + 0.5*(vth)*(width+height);
        goal_vel[2] = -vy - vx + 0.5*(vth)*(width+height);
        goal_vel[3] = -vy + vx + 0.5*(vth)*(width+height);
    }*/
    
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
      
      //2自由度PID制御実行
      speed_now[i] = (enc_delta[i]/enc_resolution)*2*pi*wheel_radius/dt; //回転速度の計算
      //changeGains(PID[i], vx-vy_pre, vy-vy_pre, i); //加減速時はPIDのゲインを変更
      motor_pwm = (int)PID[i]->calcValue(goal_vel[i] - speed_now[i], goal_vel[i]/wheel_radius);
      //motor_pwm = 255.0*goal_vel[i];
      if(fabs(speed_now[i])<0.0001 && goal_vel[i]==0) motor_pwm = 0; //高周波音の抑制 
      *md[i] << motor_pwm; //モーターを回転させる
    }
    
    //計算した値をパブリッシュ
    #ifdef ROS
    enc_msg.e1 = speed_now[0];
    enc_msg.e2 = speed_now[1];
    enc_msg.e3 = speed_now[2];
    enc_msg.e4 = speed_now[3];
    enc_pub.publish(&enc_msg);
    /*pid_msg.x = PID[0]->getP();
    pid_msg.y = PID[0]->getI();
    pid_msg.z = PID[0]->getD();
    pid_pub.publish(&pid_msg);*/
    #endif
    
    #ifdef ROS
    //debug_data.data = goal_vel[1];//(t_now - t_pre)/1000;
    //debug_pub.publish(&debug_data);
    //t_pre = t_now;
    nh.spinOnce();
    #endif
    
    time_prev = time_current;
  }
  else if(time_current - time_prev >= dt*1000000.0 && !nh.connected()){
     for(int i=0; i<MOTOR_NUM; i++) *md[i] << 0; //ROSに接続されていない間は、停止させる
     time_prev = time_current;
  }
}

void onReceivePower(const abu2021_msgs::motor_pw &v) {
  //目標速度の設定
  /*vx_raw = v.vx;
  vy_raw = v.vy;
  vth_raw = v.w;*/

  goal_vel[0] = v.v1;
  goal_vel[1] = v.v2;
  goal_vel[2] = v.v3;
  goal_vel[3] = v.v4;

}

/*void changeGains(PIDCalculator *c, float accx, float accy, int id){
  float extra_k = 50;
  //加減速度が発生している時は、ゲインを変更する
  switch(id){
    case 0:
    c->kp = kp[0] + extra_k*(step_func(-accx) + step_func(accy));
    break;
    
    case 1:
    c->kp = kp[1] + extra_k*(step_func(-accx) + step_func(-accy));
    break;
    
    case 2:
    c->kp = kp[2] + extra_k*(step_func(accx) + step_func(-accy));
    break;
    
    case 3:
    c->kp = kp[3] + extra_k*(step_func(accx) + step_func(accy));
    break;
  }
}

//階段関数
float step_func(float val){
  return (val>0.00001 ? 1.0 : 0.0);
}*/
