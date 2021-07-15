#include <ros/ros.h>
#include <math.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Joy.h>
#include <abu2021_msgs/cmd_vw.h>
#include <abu2021_msgs/auto_drive_order.h>
#include <abu2021_msgs/turn_and_dist.h>

#define but1 msg->buttons[0]
#define but2 msg->buttons[1]
#define but3 msg->buttons[2]
#define but4 msg->buttons[3]
#define butL msg->buttons[4]
#define butR msg->buttons[5]
#define butZL msg->buttons[6]
#define butZR msg->buttons[7]

double TURN_R = 1; //回転台操作の旋回半径[m]

ros::Publisher pub;
ros::Publisher pub_ad;
ros::Publisher pub_turn_and_dist;

void get_joy(const sensor_msgs::Joy::ConstPtr& msg);

int main(int argc, char **argv){
	ros::init(argc, argv, "task_manager_DR");

	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("joy", 1, get_joy);
	pub = nh.advertise<abu2021_msgs::cmd_vw>("cmd", 1);
	pub_ad = nh.advertise<abu2021_msgs::auto_drive_order>("ad_order", 1);
	pub_turn_and_dist = nh.advertise<abu2021_msgs::turn_and_dist>("turn_and_dist_order", 1);

	ros::spin();

	return 0;
}

// ジョイコン
void get_joy(const sensor_msgs::Joy::ConstPtr& msg){
	abu2021_msgs::cmd_vw cmd;
	abu2021_msgs::auto_drive_order ad_order;
	abu2021_msgs::turn_and_dist t_d;

	if(butR == 0){ //手動走行
		cmd.vx = 2.0*msg->axes[1];
		cmd.vy = 2.0*msg->axes[0];
		cmd.w  = 3.0*msg->axes[3];
	}else if(butR == 1){ //回転台操作走行
		cmd.vx = 2.0*msg->axes[2];
		cmd.vy = 0;
		cmd.w  = -2.0*msg->axes[2]/TURN_R;
	}

	//自動走行
	ad_order.go = but4;
	ad_order.emg_stop = butL;
	if(butL == 1){
		if     (but1 == 1) ad_order.set1 = 1;
		else if(but2 == 1) ad_order.set2 = 1;
		else if(but3 == 1) ad_order.set3 = 1;
	}
	//経路設定
	else if(but1 == 1) ad_order.path1 = 1;
	else if(but2 == 1) ad_order.path2 = 1;
	else if(but3 == 1) ad_order.path3 = 1;
	else if(but4 == 1) ad_order.path4 = 1;

	//回転台操作
	if(butZR == 1) t_d.solenoid_r = 1;
	else t_d.solenoid_r = 0;
	if(butZL == 1) t_d.solenoid_l = 1;
	else t_d.solenoid_l = 0;
	//妨害
	if(butR == 1) t_d.dist = 1;
	else t_d.dist = 0;

	pub.publish(cmd);
	pub_ad.publish(ad_order);
	pub_turn_and_dist.publish(t_d);
}

