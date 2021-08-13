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

//parameter
const int FREQ = 100; //[Hz]
int position3=4000; //[deg]
int position4=7000; //[deg]
int pos3_range = 300; //[deg]
int pos4_range = 500; //[deg]
double delay_hassya = 1000; //[msec] 発射時シリンダ発射状態の維持時間
int speed=150;//指定の角度に行くときのスピード

int pw=0;
int order_pow=0;
int order_set=0;
int order_mode=0;
int hassya=0;
int pre_pw=-1;
int pre_hassya=-1;
int pre_set=-1;
int pre_mode=-1;
double kakudo=0;
double std_kakudo=0;

std_msgs::Int32MultiArray pub_msg;
ros::Publisher  pub;
ros::Subscriber sub;
ros::Subscriber sub2;
/////////////modeについて////////////////////////////////////////////////////////////

//止まるモード
void mode0(){
	hassya=0;
	pw=0;
}

//手動角度調整モード
void mode1(){
	pw=order_pow;
}

//発射するモード
void mode2(){
	pw=0;
	static int cnt=0;
	
	/* ROS_FATAL("cnt: %d", cnt); */
	if(cnt > (delay_hassya+100)*FREQ/1000){
		hassya=0;
		cnt=0;
		order_mode=3; //make ready to launch again
	}else if(cnt > delay_hassya*FREQ/1000){
		hassya=2;
	}else if(cnt > 100*FREQ/1000){ //100msec
		hassya=0;
	}else if(cnt >= 0){
		hassya=1;
		 //ROS_FATAL("hassya!!"); 
	}

	cnt++;
}

//指定の角度3に行く
void mode3(){
	if(kakudo-std_kakudo >= position3+pos3_range ){
		pw=speed;//下がっていく
	}
	if(kakudo-std_kakudo <= position3-pos3_range ){
		pw=-speed;//上がっていく
	}
	if(kakudo-std_kakudo < position3+pos3_range && kakudo-std_kakudo > position3-pos3_range){
		pw=0;
		order_mode=0;
	}
}

//指定の角度4に行く
void mode4(){
	if(kakudo-std_kakudo >= position4+pos4_range ){
		pw=speed;//下がっていく
	}
	if(kakudo-std_kakudo <= position4-pos4_range ){
		pw=-speed;//上がっていく
	}
	if(kakudo-std_kakudo < position4+pos4_range && kakudo-std_kakudo > position4-pos4_range){
		pw=0;
		order_mode=0;
	}
}


/////////////setに対して///////////////////////////////////
void set(){
	if(pre_set==-1 || pre_set != order_set || pre_mode !=order_mode){
		//modeが変わって101になったか、setが変わって101になったか、初めてのときしかsetできない
		std_kakudo=kakudo;
		ROS_FATAL("SET!%f",std_kakudo);
	}
}


//コールバック関数
void chatterCallback(const std_msgs::Float64::ConstPtr & sub_msg){
	//ROS_FATAL("I heeard enncorder");
	kakudo=sub_msg->data;
}
void chatterCallback2(const abu2021_msgs::air_launch_order::ConstPtr & sub_msg2){
	/* ROS_FATAL("I herd order %d  %d  %f",sub_msg2->mode,sub_msg2->pow , std_kakudo); */
	order_mode=sub_msg2->mode;
	order_pow=sub_msg2->pow;
	order_set=sub_msg2->set;

}

int main(int argc, char** argv){
	ros::init(argc, argv, "air_launch_node");
	ros::NodeHandle nh;
	//ros::NodeHandle nhp("~");
	pub = nh.advertise<std_msgs::Int32MultiArray>("air_launch_tpc", 1);
	//ros::Subscriber sub = nh.subscribe("air_launch_enc", 1, chatterCallback);
	sub = nh.subscribe("air_launch_enc", 1, chatterCallback);
	sub2 = nh.subscribe("air_launch_order", 1, chatterCallback2);
	/*nhp*/while(!nh.getParam("air/position3", position3));
	/*nhp*/while(!nh.getParam("air/position4", position4));
	/*nhp*/while(!nh.getParam("air/pos3_range", pos3_range));
	/*nhp*/while(!nh.getParam("air/pos4_range", pos4_range));
	/*nhp*/while(!nh.getParam("air/delay_hassya", delay_hassya));
	/*nhp*/while(!nh.getParam("air/speed", speed));
	ros::Rate loop_rate(FREQ);

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

		/* ROS_FATAL("%d, %d, %d, %d", pw, pre_pw, hassya, pre_hassya); */
		//publish
		if(pw != pre_pw || hassya != pre_hassya){
			pub_msg.data[0] = pw;
			pub_msg.data[1] = hassya;
			pub.publish(pub_msg);
		}
		pre_pw = pw;
		pre_hassya = hassya;
		pre_set = order_set;
		pre_mode = order_mode;

		loop_rate.sleep();
	}
}

