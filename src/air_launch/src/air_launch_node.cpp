//カスタムメッセージ　air_launch_order.msgでタスクマネージャーから
//air_launch_ordderをサブスクライブ
//air_launch_encをサブスクライブ、air_launch_to_arduinoにパブリッシュ

/*
mode：
　０：緊急停止（モータはその場で停止，シリンダはひっこめる）
　１：角度変更（手動）
　２：射出
　３～：指定の角度（パラメータ）にする
pow：modeが1のとき，powのパワーでモータを動かして，射出を上げ下げする
set:1のときその角度を基準角度に
*/
#include <ros/ros.h>
#include <abu2021_msgs/air_launch_order.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64.h>

int position3=4000;
int position4=7000;
double kakudo=0;
double std_kakudo=0;
int pw=0;
int order_pow=0;
int order_set=0;
int speed=150;//指定の角度に行くときのスピード
int order_mode=0;
int hassya=0;

std_msgs::Int32MultiArray pub_msg;
ros::Publisher  pub;
ros::Subscriber sub;
ros::Subscriber sub2;
/////////////modeについて////////////////////////////////////////////////////////////

//止まるモード
void mode0(){
	hassya=0;
	pw=0;
	pub_msg.data[0] = pw;
	pub_msg.data[1] = hassya;
	pub.publish(pub_msg);
}

//手動角度調整モード
void mode1(){
	pw=order_pow;
}

//発射するモード
void mode2(){
	hassya=2;
	pub_msg.data[1] = hassya;
	pub.publish(pub_msg);
	ROS_FATAL("hassya!!");
	/*
	hassya=0;
	pub_msg.data[1] = hassya;
	pub.publish(pub_msg);
	hassya=1;
	pub_msg.data[1] = hassya;
	pub.publish(pub_msg);
	hassya=0;
	pub_msg.data[1] = hassya;
	ROS_FATAL("hassya!!");
	pub.publish(pub_msg);
	*/
	//ここにdelay的なものを入れてすぐにorder0にならないようにしないとどうやら射出できないぽい、かといってmodeを変えないと永遠に射出が動く
	//order_mode=0;

}

//指定の角度3に行く
void mode3(){
	hassya=0;
	if(kakudo-std_kakudo >= position3+300 ){
		pw=speed;//下がっていく
	}
	if(kakudo-std_kakudo <= position3-300 ){
		pw=-speed;//上がっていく
	}
	if(kakudo-std_kakudo < position3+300 && kakudo-std_kakudo > position3-300){
		pw=0;
	}
}

//指定の角度4に行く
void mode4(){
	hassya=0;
	if(kakudo-std_kakudo >= position4+500 ){
		pw=speed;//下がっていく
	}
	if(kakudo-std_kakudo <= position4-500 ){
		pw=-speed;//上がっていく
	}
	if(kakudo-std_kakudo < position4+500 && kakudo-std_kakudo > position4-500){
		pw=0;
	}
}


/////////////setに対して///////////////////////////////////
void set(){
	hassya=0;
	std_kakudo=kakudo;
}


//コールバック関数
void chatterCallback(const std_msgs::Float64::ConstPtr & sub_msg){
	//ROS_FATAL("I heeard enncorder");
	kakudo=sub_msg->data;
}
void chatterCallback2(const abu2021_msgs::air_launch_order::ConstPtr & sub_msg2){
	ROS_FATAL("I herd order %d  %d  %f",sub_msg2->mode,sub_msg2->pow , std_kakudo);
	order_mode=sub_msg2->mode;
	order_pow=sub_msg2->pow;
	order_set=sub_msg2->set;

}

int main(int argc, char** argv){
	ros::init(argc, argv, "air_launch_node");
	ros::NodeHandle nh;
	pub = nh.advertise<std_msgs::Int32MultiArray>("air_launch_tpc", 1);
	//ros::Subscriber sub = nh.subscribe("air_launch_enc", 1, chatterCallback);
	sub = nh.subscribe("air_launch_enc", 1, chatterCallback);
	sub2 = nh.subscribe("air_launch_order", 1, chatterCallback2);
	ros::Rate loop_rate(10);
	while (ros::ok()){
		ros::spinOnce();
		//std_msgs::Int32MultiArray pub_msg;
		pub_msg.data.resize(2);

		//ROS_INFO("I am air_launch_node");
		if(order_mode==0) mode0();
		if(order_mode==1) mode1();
		if(order_mode==2) mode2();
		if(order_mode==3) mode3();
		if(order_mode==4) mode4();
		if(order_mode==1 && order_set==1) set();
		hassya=0;

		pub_msg.data[0] = pw;
		pub_msg.data[1] = hassya;
		pub.publish(pub_msg);
		loop_rate.sleep();
	}
}

