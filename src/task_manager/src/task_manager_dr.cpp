#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <sstream>

void get_joy(const sensor_msgs::Joy& msg);
void chatterCallback(const std_msgs::String::ConstPtr& msg);

int main(int argc, char **argv){
	ros::init(argc, argv, "task_manager_DR");

	ros::NodeHandle nh;

	/* ros::Publisher  pub = n.advertise<std_msgs::String>("chatter", 1); */
	ros::Subscriber sub = nh.subscribe("joy", 1, get_joy);

	ros::Rate loop_rate(10);

	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

void get_joy(const sensor_msgs::Joy& msg){
	ROS_FATAL("UD: [%f]", msg.axes[1]);
	ROS_FATAL("RL: [%f]", msg.axes[0]);
	ROS_FATAL("YAW: [%f]", msg.axes[2]);
}
