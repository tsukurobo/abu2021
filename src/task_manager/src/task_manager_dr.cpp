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

	cmd.vx = 2.0*msg->axes[1];
	cmd.vy = 2.0*msg->axes[0];
	cmd.w  = 3.0*msg->axes[2];

	ad_order.go = msg->buttons[4];
	ad_order.emg_stop = msg->buttons[5];
	if     (msg->buttons[5]==1 && msg->buttons[2]==1) ad_order.set1 = 1;
	else if(msg->buttons[5]==1 && msg->buttons[3]==1) ad_order.set2 = 1;
	else if(msg->buttons[5]==1 && msg->buttons[0]==1) ad_order.set3 = 1;
	else if(msg->buttons[2]==1) ad_order.path1 = 1;
	else if(msg->buttons[3]==1) ad_order.path2 = 1;
	else if(msg->buttons[0]==1) ad_order.path3 = 1;
	else if(msg->buttons[1]==1) ad_order.path4 = 1;

	pub.publish(cmd);
	pub_ad.publish(ad_order);
}

