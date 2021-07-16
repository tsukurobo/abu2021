#include <ros/ros.h>
#include <abu2021_msgs/cmd_vw.h>
#include <abu2021_msgs/motor_pw.h>
#include <std_msgs/Float32.h>
#include <math.h>

#define SQRT2 1.41421356
#define MOTOR_NUM 4
#define MODE_DR 0
#define MODE_TR 1

int mode = 0; //デフォルトはDRモード
double h=1.0, w=1.0, dtoc=1.0, max_lin_v=2.0;// max_ang_v=1.0;
double vx_raw = 0, vy_raw = 0, vth_raw = 0, gyro_m = 0;

void velcallback(const abu2021_msgs::cmd_vw& vc){
    vx_raw = vc.vx;
    vy_raw = vc.vy;
    vth_raw = vc.w;
    double norm_v = sqrt(vx_raw*vx_raw+vy_raw*vy_raw);
    //並進速度司令の大きさの最大値を制限
    if(norm_v > max_lin_v){
        vx_raw = max_lin_v*vx_raw/norm_v;
        vy_raw = max_lin_v*vy_raw/norm_v;
    }

    /*if(mode == "DR"){
        //4輪オムニ
        v_raw[0] = 0.5*SQRT2*vy + 0.5*SQRT2*vx + dtoc*vc.w;
        v_raw[1] = 0.5*SQRT2*vy - 0.5*SQRT2*vx + dtoc*vc.w;
        v_raw[2] = -0.5*SQRT2*vy - 0.5*SQRT2*vx + dtoc*vc.w;
        v_raw[3] = -0.5*SQRT2*vy + 0.5*SQRT2*vx + dtoc*vc.w;
    }
    else if(mode == "TR"){
        //メカナム
        v_raw[0] = vy + vx + 0.5*(vc.w)*(w+h);
        v_raw[1] = vy - vx + 0.5*(vc.w)*(w+h);
        v_raw[2] = -vy - vx + 0.5*(vc.w)*(w+h);
        v_raw[3] = -vy + vx + 0.5*(vc.w)*(w+h);
    }*/
}

void gyroCallback(const std_msgs::Float32& wm){
    gyro_m = wm.data;
}

int main(int argc, char **argv){
    //ROSの初期化
    ros::init(argc, argv, "kinematics_model");

    //変数定義
    ros::NodeHandle nh;
    //ros::NodeHandle nhp("~"); //パラメータ読み込み用ノードハンドラ
    abu2021_msgs::motor_pw mpw; //各モータの回転速度
    double acc=2.0, freq=50.0; //ループの周波数(デフォルト50hz)
    double vx = 0, vy = 0, vth = 0, vx_pre = 0, vy_pre = 0, vth_pre = 0, motor_v[MOTOR_NUM]={0}, v_now[MOTOR_NUM] = {0}, v_pre[MOTOR_NUM]={0}; //前回のループの値
    double gyro_ki = 0, gyro_kp = 0, robot_th = 0, robot_th_sum = 0;
    //パラメータの読み込み
    while(!nh.getParam("/base/model/mode",mode));
    if(mode == MODE_TR){
        while(!nh.getParam("/base/model/width", w));
        while(!nh.getParam("/base/model/height", h));
    }
    else if(mode == MODE_DR){
        while(!nh.getParam("/base/model/distance_to_center", dtoc));
    }

    while(!nh.getParam("/base/model/acc", acc));
    while(!nh.getParam("/base/model/freq", freq));
    while(!nh.getParam("/base/gyro/kp", gyro_kp));
    while(!nh.getParam("/base/gyro/ki", gyro_ki));
    ros::Rate loop_rate(freq);

    //パブリッシャとサブスクライバをつくる
    ros::Publisher vpub = nh.advertise<abu2021_msgs::motor_pw>("motor_vel", 100);
    ros::Subscriber vsub = nh.subscribe("cmd", 100, velcallback);
    ros::Subscriber gyrosub = nh.subscribe("gyro", 100, gyroCallback);

    //メインループ
    while(ros::ok()){
        ros::spinOnce();
        
        /*for(int i=0; i<MOTOR_NUM; i++){
          if(v_now[i] < v_raw[i]) v_now[i] = v_pre[i] + acc*(1.0/freq);
          else if(v_now[i] > v_raw[i]) v_now[i] = v_pre[i] - acc*(1.0/freq);

          if((v_now[i]-v_raw[i])*(v_pre[i]-v_raw[i]) <= 0) v_now[i] = motor_v[i];
        }*/

        if(vx < vx_raw) vx = vx_pre + acc*(1.0/freq);
        else if(vx > vx_raw) vx = vx_pre - acc*(1.0/freq);
        if((vx - vx_raw)*(vx_pre - vx_raw) <= 0) vx = vx_raw;

        if(vy < vy_raw) vy = vy_pre + acc*(1.0/freq);
        else if(vy > vy_raw) vy = vy_pre - acc*(1.0/freq);
        if((vy - vy_raw)*(vy_pre - vy_raw) <= 0) vy = vy_raw;

        /*if(vth < vth_raw) vth = vth_pre + acc*(1.0/freq);
        else if(vth > vth_raw) vth = vth_pre - acc*(1.0/freq);
        if((vth - vth_raw)*(vth_pre - vth_raw) <= 0) vth = vth_raw;*/

        /*角度修正システム
        加速中かつ回転速度司令が0rad/sのときに動作させる*/
        if((fabs(vx-vx_pre) > 0.001 || fabs(vy-vy_pre) > 0.001) && fabs(vth_raw) < 0.001){
            robot_th += gyro_m*(1.0/freq);
            robot_th_sum += robot_th;
            vth = -(gyro_kp*robot_th + gyro_ki*robot_th_sum); 
        }else{
            robot_th = 0;
            robot_th_sum = 0;
            vth = vth_raw;
        }

        vx_pre = vx;
        vy_pre = vy;
        //vth_pre = vth;

        if(mode == MODE_DR){
            //4輪オムニ
            motor_v[0] = 0.5*SQRT2*vy + 0.5*SQRT2*vx + dtoc*vth;
            motor_v[1] = 0.5*SQRT2*vy - 0.5*SQRT2*vx + dtoc*vth;
            motor_v[2] = -0.5*SQRT2*vy - 0.5*SQRT2*vx + dtoc*vth;
            motor_v[3] = -0.5*SQRT2*vy + 0.5*SQRT2*vx + dtoc*vth;
        }
        else if(mode == MODE_TR){
        //メカナム
            motor_v[0] = vy + vx + 0.5*(vth)*(w+h);
            motor_v[1] = vy - vx + 0.5*(vth)*(w+h);
            motor_v[2] = -vy - vx + 0.5*(vth)*(w+h);
            motor_v[3] = -vy + vx + 0.5*(vth)*(w+h);
        }


        /*mpw.v1 = v_now[0];
        mpw.v2 = v_now[1];
        mpw.v3 = v_now[2];
        mpw.v4 = v_now[3];
        vpub.publish(mpw);
        for( int i=0; i<MOTOR_NUM; i++) v_pre[i] = v_now[i];*/

        mpw.v1 = motor_v[0];
        mpw.v2 = motor_v[1];
        mpw.v3 = motor_v[2];
        mpw.v4 = motor_v[3];
        vpub.publish(mpw);

        loop_rate.sleep();
    }
    return 0;
}
