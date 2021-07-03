#include "ros/ros.h"
/* 送信内容
 * ・サーボモータの角度×２
 * ・ソレノイドの状態(0:LOW,LOW 1:LOW,HIGH(開ける) 2:HIGH,LOW(閉める))
 * →Int32MultiArrayの中身：サーボ１の角度、サーボ２の角度、ソレノイドの状態
 */
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32.h"
#include <sstream>

std_msgs::Int32MultiArray ard_order;
std_msgs::Int32 task_order;

ros::Publisher pub_ard;
ros::Subscriber sub_task;

int deg_ini1 = 140;
int deg_ini2 = 130;
int deg_pick1 = 160;
int deg_pick2 = 150;
int deg_load1 = 175;
int deg_load2 = 160;

int deg_servo1 = deg_ini1;
int deg_servo2 = deg_ini2;
int solenoid = 0;

int order = 0;

void all_stop();
void collect();
void publish_ard();
void get_order(const std_msgs::Int32& task_order);

int main(int argc, char **argv) {

	ros::init(argc, argv, "rack_talker");
	ros::NodeHandle nh;
	
	pub_ard = nh.advertise<std_msgs::Int32MultiArray>("rack_ard_order", 1);
	sub_task = nh.subscribe("rack_task_order", 1, get_order);
	
	nh.getParam("deg_ini1",deg_ini1);
	nh.getParam("deg_ini2",deg_ini2);
	nh.getParam("deg_pick1",deg_pick1);
	nh.getParam("deg_pick2",deg_pick2);
	nh.getParam("deg_load1",deg_load1);
	nh.getParam("deg_load2",deg_load2);

	ros::Rate loop_rate(100);

	while (ros::ok()) {
		ros::spinOnce();
		
		if (order == 0) { //停止

			all_stop();

		} else { //ラック回収

			collect();
		}

		publish_ard();

		loop_rate.sleep();

	}

	return 0;
}

void get_order(const std_msgs::Int32& task_order) {
	order = task_order.data;
}

void publish_ard() {
	ard_order.data.resize(3);
	ard_order.data[0] = deg_servo1;
	ard_order.data[1] = deg_servo2;
	ard_order.data[2] = solenoid;
	
	pub_ard.publish(ard_order);
}

void collect() {
	if (order == 1) { //アームをラックへ
		
		if (deg_servo1 != deg_pick1) {
			if (deg_servo1 < deg_pick1) deg_servo1 += 1;
			else deg_servo1 -= 1;
			solenoid = 1; //開いておく
		}

	} else if (order == 2) { //持ち手を差し込む

		if (deg_servo2 != deg_pick2) {
			if (deg_servo2 < deg_pick2) deg_servo2 += 1;
			else deg_servo2 -= 1;
			solenoid = 0; //緩めておく
		}

	} else if (order == 3) { //掴む

		solenoid = 2;

	} else if (order == 4) { //アームを射出機構へ

		if (deg_servo1 != deg_load1) {
			if (deg_servo1 < deg_load1) deg_servo1 += 1;
			else deg_servo1 -= 1;

		} else if (deg_servo2 != deg_load2) {
			if (deg_servo2 < deg_load2) deg_servo2 += 1;
			else deg_servo2 -= 1;
		}
	} else if (order == 5) { //離す

		solenoid = 1;
		all_stop();

	}
}

void all_stop() {
	order = 0;
	deg_servo1 = deg_ini1;
	deg_servo2 = deg_ini2;
	solenoid = 0;
}
