#include <vector>
#include "point.h"
#include "pure_pursuit.h"

//global variable
const double MAX_SPEED = 10; //最大移動速度[m/s]
const int AHEAD_NUM = 10; //何個先の経路点列を目指すか[･]
const double RANGE_FIN = 0.1; //終了範囲[m]
const double RANGE_DCL = 1; //減速開始範囲[m]
const double YAW_GAIN_P = 1; //yaw軸PID制御Pゲイン
const double YAW_GAIN_I = 0; //yaw軸PID制御Iゲイン
const double YAW_GAIN_D = 0; //yaw軸PID制御Dゲイン

int main(int argc, char const* argv[]){
	//初期設定
	Pure_pursuit pp("../pathes/hoge4.csv", AHEAD_NUM);
	/* pp.print_path(); */

	//逐次処理
	pp.set_state(0,0,0); //現在ロボ状態

	pp.cmd_velocity(MAX_SPEED, RANGE_FIN, RANGE_DCL);
	pp.cmd_angular_v(YAW_GAIN_P, YAW_GAIN_I, YAW_GAIN_D);
	printf("cmd vx:%f\tcmd vy:%f\n", pp.cmd_vx, pp.cmd_vy);
	printf("cmd w:%f\n", pp.cmd_w);

	//目標点の方向に指定速度で移動
	//目標店の方向にPIDで角速度司令
	//指定速度は台形制御(止まる時)(終了判定も)
	//デバッグ用に移動中か終了したかを記録
	//
	//現在の位置・姿勢を得る
	//目標方向を得る（ローカル）
	//速度司令を決定（台形）
	//角速度司令を決定（PID）
	return 0;
}
