#include "ros/ros.h"
#include "trkikou/sizi.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int32.h"
#include <sstream>

//タスクマネージャーからのトピック名 tr_order 値（0:停止、1:装填、2:射出）

std_msgs::Int64 int_enc;
std_msgs::Int32 int_order;
trkikou::sizi Sizi;

ros::Publisher chatter_sizi;
ros::Subscriber sub_enc;
ros::Subscriber sub_order;

// デフォルト値
int mode = 0; //0:pwを読む　1:pwを読まない≒PID
int deg = 75;
int pw = 120;
int solenoid = 0; //0:LOW,LOW　1:LOW,HIGH(開ける)　2:HIGH,LOW(閉める)

//long enc = 0;
int kakudo = 0;

int order = 0;

void get_enc(const std_msgs::Int64& int_enc);
void get_order(const std_msgs::Int32& int_order);

int main(int argc, char **argv) {
	
	ros::init(argc, argv, "touteki_talker");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~"); //launchファイルから引数をとるために必要
	chatter_sizi = nh.advertise<trkikou::sizi>("touteki_sizi", 1);
	sub_enc = nh.subscribe("touteki_enc", 1, get_enc);
	sub_order = nh.subscribe("tr_order", 1, get_order);
	
	//launchファイルから引数をとる。
	//ただし、 ""で囲まれた名前がlaunchに存在しない場合は実行されない。
	pnh.getParam("mode", mode); //launchファイル内のmodeという名前の値を変数modeに代入
	pnh.getParam("deg", deg);
	pnh.getParam("pw", pw);
	pnh.getParam("solenoid", solenoid);

	Sizi.mode = mode;
	Sizi.deg = deg;
	Sizi.pw = pw;
	Sizi.solenoid = solenoid;
	
  	ros::Rate loop_rate(10);
	
	while (ros::ok()) {
		ros::spinOnce();

		if (order == 0) {

			Sizi.pw = 0;
			

		} else if (order == 1) {

			if (kakudo < 0) Sizi.pw = -pw;
			else Sizi.pw = pw;
			
			
		} else if (order == 2) {
		
			

		}
			
		chatter_sizi.publish(Sizi);
		
		
    		loop_rate.sleep();
	
	}


	return 0;
}

/*
void messageCb(const std_msgs::Int64& int_msg) {
	if (int_msg.data >= 0) {
		Sizi.pw =pw;
	} else {
		Sizi.pw = -pw;
	}
}
*/

void get_enc (const std_msgs::Int64& int_enc) {
	kakudo = int_enc.data;
}

void get_order (const std_msgs::Int32& int_order) {
	order = int_order.data;
}
