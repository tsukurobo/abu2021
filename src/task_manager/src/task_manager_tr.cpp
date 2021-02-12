#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <abu2021_msgs/cmd_vw.h>

ros::Publisher pub;

void get_joy(const sensor_msgs::Joy::ConstPtr& msg);

int main(int argc, char **argv){
	ros::init(argc, argv, "task_manager_TR");

	ros::NodeHandle nh;

	pub = nh.advertise<abu2021_msgs::cmd_vw>("cmd_tr", 1);
	ros::Subscriber sub = nh.subscribe("joy", 1, get_joy);

	ros::spin();

	return 0;
}

void get_joy(const sensor_msgs::Joy::ConstPtr& msg){
	abu2021_msgs::cmd_vw cmd;

	cmd.vx = -msg.axes[0];
	cmd.vy = msg.axes[1];
	cmd.w  = msg.axes[3];

	pub.publish(cmd);
}

