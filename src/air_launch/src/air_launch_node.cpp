
#include <ros/ros.h>
//#include <abu2021_msgs/air_launch_to_arduino.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>

//air_launch_encをサブスクライブ、air_launch_to_arduinoにパブリッシュ
//air_launch_ordderをサブスクライブ


int low_position=3000;
int up_position=7000;
double kakudo=0;
int pw=0;
int speed=200;
int order=0;
int hassya=0;

std_msgs::Int32MultiArray pub_msg;
ros::Publisher  pub;
ros::Subscriber sub;
ros::Subscriber sub2;


//上げるモードある角度に行く
void mode1(){
	hassya=0;
	if(kakudo >= up_position+200 ){
		pw=speed;
		//下がっていく
	}
	if(kakudo<= up_position-200 ){
		pw=-speed;
		//上がっていく
	}

	if(kakudo<up_position+200 && kakudo>up_position-200){
		pw=0;
	}
}

//さげるモードある角度に行く
void mode2(){
	hassya=0;
	if(kakudo >= low_position+200 ){
		pw=speed;
		//下がっていく
	}
	if(kakudo<= low_position-200 ){
		pw=-speed;
		//上がっていく
	}

	if(kakudo<low_position+200 && kakudo>low_position-200){
		pw=0;
	}


}

//止まるモード
void mode0(){
	pw=0;
}


void mode3(){
	hassya=2;
	pub_msg.data[1] = hassya;
	pub.publish(pub_msg);
	hassya=0;
	pub_msg.data[1] = hassya;
	pub.publish(pub_msg);
	hassya=1;
	pub_msg.data[1] = hassya;
	pub.publish(pub_msg);
	hassya=0;
	pub_msg.data[1] = hassya;
	pub.publish(pub_msg);


}
//コールバック関数
void chatterCallback(const std_msgs::Float64::ConstPtr & sub_msg){
	//ROS_FATAL("I heeard enncorder");
	kakudo=sub_msg->data;
}
void chatterCallback2(const std_msgs::Int16::ConstPtr & sub_msg2){
	ROS_FATAL("I herd order %d",sub_msg2->data);
	order=sub_msg2->data;
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
		if(order==0) mode0();
		if(order==1) mode1();
		if(order==2) mode2();
		if(order==3) mode3();

		pub_msg.data[0] = pw;
		pub.publish(pub_msg);
		loop_rate.sleep();
	}
}

