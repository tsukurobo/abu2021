#include <ros.h>
#include <trkikou/sizi.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/Int64.h>

#include "ise_motor_driver_v3.h"

<<<<<<< HEAD
=======
#define PI 3.141592653589793
>>>>>>> 3efad891a56545346569196e91afb6ef7fda1b77
#define PIN_SOLENOID_1 2 //ピン番号
#define PIN_SOLENOID_2 6 //ピン番号

IseMotorDriver md(0x15); //AVRのアドレスに合わせて変更

ros::NodeHandle nh;

std_msgs::Int64MultiArray int_debug;
std_msgs::Int64 int_msg;

ros::Publisher chatter_debug("touteki_arduino_debug", &int_debug);
ros::Publisher chatter_enc("touteki_enc", &int_msg);

int mode;
int deg;
int pw;
int solenoid;

long enc=0;
float kakudo = 0;

<<<<<<< HEAD
float Kp = 0.6, Ki = 0.027, Kd = 0.5; //比例、積分、微分ゲイン
=======
float Kp = 3, Ki = 0.025, Kd = 0.8; //比例、積分、微分ゲイン
//float Kp = 0.55, Ki = 0.018, Kd = 0.8; //比例、積分、微分ゲイン
>>>>>>> 3efad891a56545346569196e91afb6ef7fda1b77
float P = 0, I = 0, D = 0; //比例項、積分項、微分項
float kakudo_p = 0;

void messageCb( const trkikou::sizi& Sizi) {
    int_debug.data[0] = Sizi.mode;
    int_debug.data[1] = Sizi.deg;
    int_debug.data[2] = Sizi.pw;
    int_debug.data[3] = Sizi.solenoid;
    int_debug.data[4] = 0;
    int_debug.data[5] = 0;
    int_debug.data[6] = 0;
    int_debug.data[7] = 0;
    
    mode = Sizi.mode;
    deg = Sizi.deg;
    pw = Sizi.pw;
    solenoid = Sizi.solenoid;
    
}

ros::Subscriber<trkikou::sizi> sub_sizi("touteki_sizi", messageCb);

void PID() {
    //int static kakudo_p = 0;
    
    P = Kp*(deg - kakudo);
    I += Ki*(deg - kakudo);
    D = -Kd*(kakudo - kakudo_p);

<<<<<<< HEAD
    pw = P + I + D;

    if (pw > 255) pw = 255;
    if (pw < -255) pw = -255;
=======
    pw = (P + I + D)*abs(sin(kakudo*PI/180));

    if (pw > 150) pw = 150;
    if (pw < -150) pw = -150;
>>>>>>> 3efad891a56545346569196e91afb6ef7fda1b77

    md << pw;

    kakudo_p = kakudo;

    int_debug.data[4] = pw;
    int_debug.data[5] = P;
    int_debug.data[6] = I;
    int_debug.data[7] = D;
}

void setup() {
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    
    nh.subscribe(sub_sizi); //実際に動かす
    nh.advertise(chatter_debug);
    nh.advertise(chatter_enc);
    
    int_debug.data = (long long int*)malloc(sizeof(long long int) *8);
    int_debug.data_length = 8;
  
    IseMotorDriver::begin();
    if(!md==false) md << IseMotorDriver::createSettingData(PWM_64KHZ,SM_BRAKE_LOW_SIDE);

    pinMode(PIN_SOLENOID_1, OUTPUT);
    pinMode(PIN_SOLENOID_2, OUTPUT);
    digitalWrite(PIN_SOLENOID_1, LOW);
    digitalWrite(PIN_SOLENOID_2, LOW);

}

void loop() {
    nh.spinOnce(); // トピックの更新を確認
  
    md >> enc;
    //kakudo = enc*180/2048;
    kakudo = -enc*180/2048;

    int_msg.data = kakudo;

    //モーター（mode=0:rosからの受信した一定出力,　mode=1:PID）
    if (mode == 0) {
        md << pw;

        I = 0;
        kakudo_p = 0;// PIDの初期化
    }
    
    else if (mode == 1) PID();
    
    //ソレノイド(solenoid=0:緩める, solenoid=1:開ける, solenoid=2:閉める)
    if (solenoid == 0) {
        
        digitalWrite(PIN_SOLENOID_1, LOW); // 緩める
        digitalWrite(PIN_SOLENOID_2, LOW);
        
    } else if (solenoid == 1) {
        
<<<<<<< HEAD
        digitalWrite(PIN_SOLENOID_1, LOW); //　開ける
        digitalWrite(PIN_SOLENOID_2, HIGH);
        
    } else if (solenoid == 2) {
        
        digitalWrite(PIN_SOLENOID_1, HIGH); // 閉める
        digitalWrite(PIN_SOLENOID_2, LOW);
=======
        digitalWrite(PIN_SOLENOID_1, HIGH); //　開ける
        digitalWrite(PIN_SOLENOID_2, LOW);
        
    } else if (solenoid == 2) {
        
        digitalWrite(PIN_SOLENOID_1, LOW); // 閉める
        digitalWrite(PIN_SOLENOID_2, HIGH);
>>>>>>> 3efad891a56545346569196e91afb6ef7fda1b77
        
    }
    

    chatter_debug.publish(&int_debug);
    chatter_enc.publish(&int_msg);
  
<<<<<<< HEAD
    delay(10);
=======
    delay(1);
>>>>>>> 3efad891a56545346569196e91afb6ef7fda1b77
}
