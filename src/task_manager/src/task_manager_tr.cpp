#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <abu2021_msgs/cmd_vw.h>
#include <std_msgs/Int32.h>

ros::Publisher pub;
ros::Publisher pub_touteki;

int order = 0;

void get_joy(const sensor_msgs::Joy::ConstPtr& msg);

int main(int argc, char **argv){
	ros::init(argc, argv, "task_manager_TR");

	ros::NodeHandle nh;

	pub = nh.advertise<abu2021_msgs::cmd_vw>("cmd", 1);
	pub_touteki = nh.advertise<std_msgs::Int32>("/touteki/tr_order", 1);
	ros::Subscriber sub = nh.subscribe("joy", 1, get_joy);

	ros::spin();

	return 0;
}

void get_joy(const sensor_msgs::Joy::ConstPtr& msg){
	abu2021_msgs::cmd_vw cmd;
	std_msgs::Int32 int_order;

	cmd.vx = msg->axes[1];
	cmd.vy = msg->axes[0];
	cmd.w  = msg->axes[2];
	
	if ((msg->buttons[0] == 1) || (msg->buttons[3] == 1)) order = 0;
	else if (msg->buttons[2] == 1) order = 1;
	else if (msg->buttons[1] == 1) order = 2;

	int_order.data = order;

	pub.publish(cmd);
	pub_touteki.publish(int_order);
}
