#include "ros/ros.h"
#include "abu2021_msgs/launch_act.h"
#include "abu2021_msgs/launch_sens.h"
#include "abu2021_msgs/rack_msg.h"
#include "abu2021_msgs/tr_order.h"

int FREQ = 100; //制御周期
int AIR_HIGH_TIME = 200; //[msec]
int AIR_LAUNCH_TIME = 2000; //[msec]

double sens_enc = 0;
int sens_touch = 0;
int order_air = 0;
int order_const = 0;
int order_rack = 0;
int order_stop = 0;
int step_air = 0;
int step_const = 0;
int step_rack = 0;

void task_air();
void get_launch_sensor(const abu2021_msgs::launch_sens::ConstPtr& msg);
void get_order(const abu2021_msgs::tr_order::ConstPtr& msg);

int main(int argc, char **argv){
	ros::init(argc, argv, "tr_mechanism_node");

	ros::NodeHandle nh;

	ros::Publisher  pub_launch = nh.advertise<abu2021_msgs::launch_act>("launch_act", 1);
	ros::Publisher  pub_rack   = nh.advertise<abu2021_msgs::rack_msg>("rack_tpc", 1);
	ros::Subscriber sub_sens   = nh.subscribe("launch_sens", 1, get_launch_sensor);
	ros::Subscriber sub_order  = nh.subscribe("tr_order", 1, get_order);

	/* nh.getParam("deg_ini1",deg_ini1); */

	ros::Rate loop_rate(10);

	while (ros::ok()){
		ros::spinOnce();

		if(order_air == 1 || step_air > 0){
			task_air();
		}


		loop_rate.sleep();
	}
	return 0;
}

void task_air(){
	static int cnt = 0;

	if(step_air == 0){

		step_air = 1;
	}else if(step_air == 1){
		cnt++;

		if(cnt > AIR_HIGH_TIME*FREQ/1000){

		}
	}

}


void get_launch_sensor(const abu2021_msgs::launch_sens::ConstPtr& msg){
	sens_enc = msg->enc;
	sens_touch = msg->touch;
}

void get_order(const abu2021_msgs::tr_order::ConstPtr& msg){
	order_air   = msg->air;
	order_const = msg->spring;
	order_rack  = msg->rack;
	order_stop  = msg->emg_stop;
}
