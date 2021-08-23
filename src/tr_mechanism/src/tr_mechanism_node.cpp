#include "ros/ros.h"
#include "abu2021_msgs/launch_act.h"
#include "abu2021_msgs/launch_sens.h"
#include "abu2021_msgs/rack_msg.h"

void get_launch_sensor(const abu2021_msgs::launch_sens::ConstPtr& msg);

int main(int argc, char **argv){
	ros::init(argc, argv, "tr_mechanism_node");

	ros::NodeHandle nh;

	ros::Publisher  pub_launch = nh.advertise<abu2021_msgs::launch_act>("launch_act", 1);
	ros::Publisher  pub_rack = nh.advertise<abu2021_msgs::rack_msg>("rack_tpc", 1);
	ros::Subscriber sub = nh.subscribe("launch_sens", 1, get_launch_sensor);

	/* nh.getParam("deg_ini1",deg_ini1); */

	ros::Rate loop_rate(10);

	while (ros::ok()){



		ros::spinOnce();

		loop_rate.sleep();
	}
	return 0;
}

void get_launch_sensor(const abu2021_msgs::launch_sens::ConstPtr& msg){
	ROS_FATAL("hogehoge");
}
