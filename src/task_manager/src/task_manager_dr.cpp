#include <ros/ros.h>
#include <math.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Joy.h>
#include <abu2021_msgs/cmd_vw.h>
#include <abu2021_msgs/auto_drive_order.h>

ros::Publisher pub;
ros::Publisher pub_ad;

void get_joy(const sensor_msgs::Joy::ConstPtr& msg);

int main(int argc, char **argv){
	ros::init(argc, argv, "task_manager_DR");

	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("joy", 1, get_joy);
	pub = nh.advertise<abu2021_msgs::cmd_vw>("cmd", 1);
	pub_ad = nh.advertise<abu2021_msgs::auto_drive_order>("ad_order", 1);

	ros::spin();

	return 0;
}

// ジョイコン
void get_joy(const sensor_msgs::Joy::ConstPtr& msg){
	abu2021_msgs::cmd_vw cmd;
	abu2021_msgs::auto_drive_order ad_order;

	cmd.vx = msg->axes[1];
	cmd.vy = msg->axes[0];
	cmd.w  = msg->axes[2];

	ad_order.emg_stop = msg->buttons[5];
	ad_order.path1 = msg->buttons[4];

	pub.publish(cmd);
	pub_ad.publish(ad_order);
}

