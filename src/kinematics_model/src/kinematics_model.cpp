#include <ros/ros.h>
#include <abu2021_msgs/cmd_vw.h>
#include <abu2021_msgs/motor_pw.h>
#include <string>

#define SQRT2 1.41421356
#define MOTOR_NUM 4

std::string mode = "DR"; //デフォルトはDRモード
double h=1.0, w=1.0, dtoc=1.0;//max_lin_v=1.0, max_ang_v=1.0;
double v_raw[MOTOR_NUM]={0};

void velcallback(const abu2021_msgs::cmd_vw& vc){
    //並進速度司令ベクトルの最大値を制限
    double vx = vc.vx, vy = vc.vy;// norm_v = sqrt(vx*vx+vy*vy);
    /*if(norm_v > max_lin_v){
        vx = vx/norm_v;
        vy = vy/norm_v;
    }*/
    if(mode == "DR"){
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
    }
}

int main(int argc, char **argv){
    //ROSの初期化
    ros::init(argc, argv, "kinematics_model");

    //変数定義
    ros::NodeHandle nh;
    //ros::NodeHandle nhp("~"); //パラメータ読み込み用ノードハンドラ
    abu2021_msgs::motor_pw mpw; //各モータの回転速度
    double k, k_go=1.0, k_stop=1.0, freq=100; //ループの周波数(デフォルト100hz)
    double v_pre[MOTOR_NUM]={0}; //前回のループの値
    //パラメータの読み込み
    nh.getParam("model/mode",mode);
    if(mode == "TR"){
        nh.getParam("model/width", w);
        nh.getParam("model/height", h);
    }
    else if(mode == "DR"){
        nh.getParam("model/distance_to_center", dtoc);
    }

    nh.getParam("model/LPF_const_go", k_go);
    nh.getParam("model/LPF_const_stop", k_stop);
    nh.getParam("model/freq", freq);
    ros::Rate loop_rate(freq);

    //パブリッシャとサブスクライバをつくる
    ros::Publisher vpub = nh.advertise<abu2021_msgs::motor_pw>("motor_vel", 100);
    ros::Subscriber vsub = nh.subscribe("cmd", 100, velcallback);

    //メインループ
    while(ros::ok()){
        ros::spinOnce();
        if(v_raw[0]==0 && v_raw[1]==0 && v_raw[2]==0 && v_raw[3]==0) k=k_stop;
        else k=k_go;

        mpw.v1 = k*v_raw[0] + (1-k)*v_pre[0];
        mpw.v2 = k*v_raw[1] + (1-k)*v_pre[1];
        mpw.v3 = k*v_raw[2] + (1-k)*v_pre[2];
        mpw.v4 = k*v_raw[3] + (1-k)*v_pre[3];
        vpub.publish(mpw);
        v_pre[0] = mpw.v1;
        v_pre[1] = mpw.v2;
        v_pre[2] = mpw.v3;
        v_pre[3] = mpw.v4;

        loop_rate.sleep();
    }
    return 0;
}
