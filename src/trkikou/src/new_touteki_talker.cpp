#include "ros/ros.h"
#include "trkikou/sizi.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int64MultiArray.h"
#include <sstream>

//タスクマネージャーからのトピック名 tr_order 値（0:停止、1:装填、2:射出）

std_msgs::Int64 int_enc;
std_msgs::Int32 int_order;
std_msgs::Int64MultiArray int_debug;
trkikou::sizi Sizi;

ros::Publisher chatter_sizi;
ros::Publisher chatter_debug;
ros::Subscriber sub_enc;
ros::Subscriber sub_order;

int order = 0;

// デフォルト値
int mode = 0; //0:pwを読む　1:pwを読まない≒PID
int deg = -75;
int pw = 120;
int solenoid = 0; //0:LOW,LOW　1:LOW,HIGH(開ける)　2:HIGH,LOW(閉める)

// パラメータ
int deg_pick = -75;
int deg_lift = -90;
int time_lift = 30;

int deg_launch = 470;
int deg_stop1 = 720;
int deg_slow = 260;
int deg_stop2 = 60;
int pw_launch = 120;
int pw_return = 90;


int kakudo = 0;

// 作業変数
int step_pick = 0;
int step_launch = 0;
int count_pick = 0;

// プロトタイプ宣言
void all_stop();
void pick();
void launch();
void publish_sizi();
void publish_debug();
void get_enc(const std_msgs::Int64& int_enc);
void get_order(const std_msgs::Int32& int_order);


int main(int argc, char **argv) {
	
	ros::init(argc, argv, "touteki_talker");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~"); //launchファイルから引数をとるために必要

	chatter_sizi = nh.advertise<trkikou::sizi>("touteki_sizi", 1);
	chatter_debug = nh.advertise<std_msgs::Int64MultiArray>("touteki_node_debug", 1);
	sub_enc = nh.subscribe("touteki_enc", 1, get_enc);
	sub_order = nh.subscribe("tr_order", 1, get_order);
	
	//launchファイルから引数をとる。
	//ただし、 ""で囲まれた名前がlaunchに存在しない場合は実行されない。
	pnh.getParam("deg_pick", deg_pick); //launchファイル内のdeg_pickという名前の値を変数deg_pickに代入
	pnh.getParam("deg_lift", deg_lift);
	pnh.getParam("time_lift", time_lift);
	pnh.getParam("deg_launch", deg_launch);
	pnh.getParam("deg_stop1", deg_stop1);
	pnh.getParam("deg_slow", deg_slow);
	pnh.getParam("deg_stop2", deg_stop2);
	pnh.getParam("pw_launch", pw_launch);
	pnh.getParam("pw_return", pw_return);
	
  	ros::Rate loop_rate(10);
	
	while (ros::ok()) {
		ros::spinOnce();

		if (order == 0) { //停止

			all_stop();

		} else if (order == 1) { //装填

			pick();
			
		} else if (order == 2) { //射出
		
			launch();

		}
			
		if ((kakudo > 1200) || (kakudo < -1200)) all_stop(); //セーフティ

		publish_sizi();
		publish_debug();
		
    		loop_rate.sleep();
	
	}


	return 0;
}


void get_enc (const std_msgs::Int64& int_enc) {
	kakudo = int_enc.data;
}

void get_order (const std_msgs::Int32& int_order) {
	order = int_order.data;
}

void pick () {
	if (step_pick == 0) {

		mode = 1; //PID
		deg = deg_pick; //矢を掴む位置
		solenoid = 1; //アームを開ける

		if (kakudo <= deg_pick) {
			solenoid = 0; //アームを緩める
			step_pick = 1;
		}

	} else if (step_pick == 1) {

		solenoid = 2; //アームを閉める
		sleep(2);
		step_pick = 2;
		

	} else if (step_pick == 2) {

		mode = 1; //PID
		deg = deg_lift; //少し持ち上げる 値は要検証。
		count_pick += 1;

		if (count_pick >= time_lift) {
			count_pick = 0;
			step_pick = 3;
		}

	} else if (step_pick == 3) {

		all_stop();
	}
}

void launch() {
	if (step_launch == 0) {

		mode = 0;
		pw = pw_launch;

		if (kakudo >= deg_launch) {
			solenoid = 0; //アームを緩める
			sleep(0.01);
			solenoid = 1; //射出
			step_launch = 1;
		}

	} else if (step_launch == 1) {

		pw = (kakudo-deg_stop1)*pw_launch/(deg_launch-deg_stop1); //台形制御で角度deg_stopで止める

		if (kakudo >= deg_stop1) {
			solenoid = 0; //アームを緩める
			step_launch = 2;
		}

	} else if (step_launch == 2) {

		pw = -pw_return; //戻す
		if (kakudo <= deg_slow) step_launch = 3;

	} else if (step_launch == 3) {

		pw = -(kakudo-deg_stop2)*pw_launch/(deg_slow-deg_stop2); //台形制御で角度0で止める
		if (kakudo <= deg_stop2) step_launch = 4;

	} else if (step_launch == 4) {

		all_stop();

	}
}

void publish_sizi() {
	Sizi.mode = mode;
	Sizi.deg = deg;
	Sizi.pw = pw;
	Sizi.solenoid = solenoid;

	chatter_sizi.publish(Sizi);
}

void publish_debug() {
	int_debug.data.resize(3);
	int_debug.data[0] = step_pick;
	int_debug.data[1] = step_launch;
	int_debug.data[2] = count_pick;

	chatter_debug.publish(int_debug);
}

void all_stop() {
	order = 0;

	mode = 0;
	pw = 0;
	solenoid = 0;

	step_pick = 0;
	step_launch = 0;

	count_pick = 0;
}
